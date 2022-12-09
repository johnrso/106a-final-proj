import numpy as np
import cv2
from typing import Union
import imutils

"""
 Intrinsic of "Color" / 640x480 / {YUYV/RGB8/BGR8/RGBA8/BGRA8/Y16}
  Width:      	640
  Height:     	480
  PPX:        	327.363403320312
  PPY:        	233.706436157227
  Fx:         	609.799499511719
  Fy:         	609.5458984375
  Distortion: 	Inverse Brown Conrady
  Coeffs:     	0  	0  	0  	0  	0
  FOV (deg):  	55.37 x 42.98
"""

#
# FX = 609.799499511719
# FY = 609.5458984375
# IMG_H = 480
# IMG_W = 640
# BALL_DIAM_METERS = .08
# RADIUS_MIN_SIZE = .01
#
# colorLower = (200, 0, 0)
# colorUpper = (256, 100, 100)


robot_color_min = (40, 20, 0)
robot_color_max = (190, 80, 90)

class DepthFinder:
    def __init__(self, fx, fy, h, w, ball_diam_meters, radius_min_size, color_upper, color_lower):
        """

        fx: Pixel focal length used to convert from world coords to image coords... as in X = x * fx / Z
        fy: Pixel focal length
        h: height of images in pixels
        w: width of images in pixels
        ball_diam_meters: diameter of the ball
        radius_min_size: threshold for us to count the image patch as a found ball... in meters
        color_upper: Tuple of color in rgb
        color_lower: Tuple of color in rgb
            Any pixel value with r, g, and b values in between color_upper and color_lower will be considered
            as potentially being part of the ball

        """
        self.fx = fx
        self.fy = fy
        self.h = h
        self.w = w
        self.ball_diam_meters = ball_diam_meters
        self.radius_min_size = 15
        self.color_upper = color_upper
        self.color_lower = color_lower

    def detect_from_color(self, img: np.ndarray, depth: np.ndarray, use_depth = True) -> Union[float, float, float]:
        """
        [h, w, 3] img
        [h, w] or [h, w, 1] depth... can be ignored with use_depth=False

        returns x, y, z, depth_found
        """

        assert img.shape == (self.h, self.w, 3)
        if use_depth and len(depth.shape) == 3:
          depth = depth.reshape(depth.shape[:2])
          assert img.shape[:2] == depth.shape[:2]


        blurred = cv2.GaussianBlur(img, (11, 11), 0)

        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(blurred, self.color_lower, self.color_upper)
        robot_mask = 255 - cv2.inRange(blurred, robot_color_min, robot_color_max)
        assert robot_mask.shape == mask.shape

        mask = np.clip(robot_mask * mask, 0, 255)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)#.astype(bool)



        #mask = np.logical_and(mask, robot_mask.astype(bool))

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((pix_x, pix_y), pixel_radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if pixel_radius > self.radius_min_size:
                # pixel_diameter = pixel_radius * 2
                # degrees_per_pixel_x = FOV_X / IMG_W
                # sin_theta = np.sin(np.deg2rad(pixel_radius * degrees_per_pixel_x))
                # print(pixel_radius * degrees_per_pixel_x)
                Z = self.fx * self.ball_diam_meters / (pixel_radius * 2)# (BALL_DIAM_METERS / 2) / sin_theta
                circle_mask = np.zeros(img.shape[:2])
                circle_mask = cv2.circle(circle_mask, (int(pix_x), int(pix_y)), int(pixel_radius), 1, -1)
                if use_depth:
                  partitioned = np.partition(depth[circle_mask.squeeze() > 0].squeeze(), 10)
                  z_depth = np.mean(partitioned[:10]) + (self.ball_diam_meters / 2) # Take the mean of 10 smallest depths for robustness
                  print(f"classical: {Z}, realsense {z_depth}")
                  Z = (Z + z_depth) / 2 # Average the 2 predictions
                X = (pix_x - (self.w / 2)) * Z / self.fx
                Y = (pix_y - (self.h / 2)) * Z / self.fy

                if len(circle_mask.shape) == 2:
                  circle_mask = np.stack([circle_mask] * 3, axis = -1)

                circle_mask = circle_mask * img
                # Camera frame has depth = x, left of image is y, up in image is z
                if len(mask.shape) == 2:
                    mask = np.stack([mask] * 3, axis = -1)
                return Z, -X, -Y, True, circle_mask.astype(np.uint8) * 255, mask.astype(np.uint8) * 255

        return 0, 0, 0, False, None, None
