import rospy
from geometry_msgs.msg import PoseStamped, Pose
import numpy as np

def talker():

    # Create an instance of the rospy.Publisher object which we can  use to
    # publish messages to a topic. This publisher publishes messages of type
    # std_msgs/String to the topic /chatter_talk
    pub_paths = rospy.get_param("~publishers")
    sub_paths = rospy.get_param("~subscribers")

    pub = rospy.Publisher(sub_paths["ball_pose"], PoseStamped, queue_size=100)

    # Create a timer object that will sleep long enough to result in a 10Hz
    # publishing rate
    r = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        x, y, z = np.random.sample(3)
        pos = PoseStamped()
        pos.pose.position.x = x
        pos.pose.position.y = y
        pos.pose.position.z = z
        pub.publish(pub)
        print(rospy.get_name() + ": I sent \"%s\"" % pos)
        r.sleep()

if __name__ == '__main__':

    # Run this program as a new node in the ROS computation graph called /talker.
    rospy.init_node('talker', anonymous=True)

    # Check if the node has received a signal to shut down. If not, run the
    # talker method.
    try:
        talker()
    except rospy.ROSInterruptException: pass