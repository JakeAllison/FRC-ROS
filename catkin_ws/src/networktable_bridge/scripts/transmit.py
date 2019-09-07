#!/usr/bin/env python

import threading
import rospy
from networktables import NetworkTables
from geometry_msgs.msg import Pose

IP = 'localhost'

ros_table = None

#Threading stuff for connecting to NT
cond = threading.Condition()
notified = [False]

def connectionListener(connected, info):
    print(info, '; Connected=%s' % connected)
    with cond:
        notified[0] = True
        cond.notify()

def add_to_nt(data, topic):
    ros_table.putRaw(topic, data)

def transmit():
    while True:
        ros_table.putNumber('testing', 5)
        topics = rospy.get_published_topics()

if __name__ == '__main__':
    try:
        NetworkTables.initialize(server=IP)
        NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

        with cond:
            print("Waiting")
            if not notified[0]:
                cond.wait()

        #Start transmitting ROS data to NT
        print("Connected!")
        ros_table = NetworkTables.getTable('ROS')

        rospy.init_node('network_table_bridge', anonymous=False)

        #Loop here and check if special NT entry is different than last, if so, subscribe to that topic?

        my_topic = 'chatter'
        rospy.Subscriber(my_topic, Pose, add_to_nt, callback_args=my_topic)

        transmit()
    except rospy.ROSInterruptException:
        pass