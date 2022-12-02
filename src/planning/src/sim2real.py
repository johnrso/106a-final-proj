#!/usr/bin/env python
# 15hz cam
import sys
import os
import time

import rospy
from rospy.numpy_msg import numpy_msg
import numpy as np
import ros_numpy

import torch
from torchvision import transforms

from pix2pix.options.base_options_simple import BaseOptionsSimple
from pix2pix.models import create_model

from sensor_msgs.msg import Image


class Sim2RealNode(object):
    def __init__(self):
        rospy.init_node("sim2real", anonymous=True)

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        print(f"using device: {self.device}")
        self.transforms = transforms.Compose(
            [transforms.CenterCrop(440), transforms.Resize(256)]
        )
        self.p2p_config = rospy.get_param("~pix2pix")

        print(f"cwd: {os.getcwd()}")
        # load model
        self.ckpt_path = self.p2p_config["ckpt_path"]
        opt = BaseOptionsSimple().parse(self.p2p_config["hps"].split(" "))
        opt.num_threads = self.p2p_config["num_threads"]
        opt.batch_size = self.p2p_config["batch_size"]
        opt.serial_batches = self.p2p_config["serial_batches"]
        opt.no_flip = self.p2p_config["no_flip"]
        opt.display_id = self.p2p_config["no_flip"]
        self.p2p = create_model(opt)
        try:
            self.p2p.custom_load(self.ckpt_path)
        except:
            print(
                "model ckpt likely not found; please use an absolute path in config/sim2real.yaml."
            )
            raise

        # setup rostopic
        self.img_pub_path = rospy.get_param("~publishers/img_map")
        self.seg_pub_path = rospy.get_param("~publishers/seg_map")
        self.crop_pub_path = rospy.get_param("~publishers/crop_map")
        self.sub_path = rospy.get_param("~subscribers/images")
        self.img_pub = rospy.Publisher(self.img_pub_path, Image, queue_size=1)
        self.seg_pub = rospy.Publisher(self.seg_pub_path, Image, queue_size=1)
        self.crop_pub = rospy.Publisher(self.crop_pub_path, Image, queue_size=1)

        rospy.Subscriber(self.sub_path, Image, self.sim2real)

        rospy.loginfo("sim2real initialized")

        if rospy.get_param("~test"):
            self.test()

        # spin
        rospy.spin()

    def sim2real(self, img_msg):
        """
        publishes a simulated image

        input img: 3xHxW RGB image. automatically cropped into 3x256x256.
        output none
        """
        start = time.time()

        # preprocessing; crop to 256, still needs normalization. add + remove batching for p2p model
        img_mat = ros_numpy.numpify(img_msg)
        img = (
            torch.from_numpy(np.transpose(img_mat[:, :, ::-1].copy(), (2, 0, 1)))
            .unsqueeze(0)
            .type(torch.FloatTensor)
        )
        img = img / 255.0
        img = img[:, :, :440, 200:]
        img = self.transforms(img).to(self.device)
        noise = torch.zeros_like(img).to(self.device)

        self.p2p.set_input(
            {"A": img, "B": noise, "D": noise, "A_paths": noise, "B_paths": noise}
        )
        self.p2p.test()

        segmap = torch.transpose(self.p2p.fake_B.squeeze(0), -1, -3)
        segmap = (
            torch.argmax(segmap, dim=-1, keepdim=True).cpu().numpy().astype(np.uint8)
        )
        seg_msg = ros_numpy.msgify(Image, segmap, "mono8")
        seg_msg.header.stamp = img_msg.header.stamp
        self.seg_pub.publish(seg_msg)
        end = time.time()
        rospy.loginfo_throttle(5, f"sim2real: translated img in {end-start}")

        # extra visualizations
        if rospy.get_param("~test"):
            visuals = self.p2p.get_current_visuals()
            pix = (
                ((visuals["fake_B"] + 1) / 2 * 255)
                .cpu()
                .numpy()
                .squeeze(0)
                .astype(np.uint8)
                .transpose((1, 2, 0))
            )
            crop = (
                ((visuals["real_A"] + 1) / 2 * 255)
                .cpu()
                .numpy()
                .squeeze(0)
                .astype(np.uint8)
                .transpose((1, 2, 0))
            )
            seg_rgb_msg = ros_numpy.msgify(Image, pix, "rgb8")
            seg_rgb_msg.header.stamp = img_msg.header.stamp
            crop_msg = ros_numpy.msgify(Image, crop, "rgb8")
            crop_msg.header.stamp = img_msg.header.stamp

            self.img_pub.publish(seg_rgb_msg)
            self.crop_pub.publish(crop_msg)

    def test(self):
        img = torch.zeros((1, 3, 256, 256)).to(self.device)
        img = transforms.CenterCrop(256)(img).to(self.device)
        noise = torch.zeros_like(img).to(self.device)

        start = time.time()
        self.p2p.set_input(
            {"A": img, "B": noise, "D": noise, "A_paths": noise, "B_paths": noise}
        )
        self.p2p.test()
        end = time.time()

        visuals = self.p2p.get_current_visuals()
        pix = visuals["fake_B"].cpu().numpy().squeeze(0)
        rospy.loginfo(f"test: out shape={pix.shape}; elapsed time={end-start}")


if __name__ == "__main__":
    try:
        Sim2RealNode()
    except rospy.ROSInterruptException:
        pass
