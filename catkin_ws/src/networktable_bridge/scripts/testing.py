#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose

pose = Pose()

def talker():
    pub = rospy.Publisher('chatter', Pose, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pose.position.x = pose.position.x + 1
        pose.position.y = pose.position.y + 1
        pose.position.z = pose.position.z + 1

        rospy.loginfo(pose)
        pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass