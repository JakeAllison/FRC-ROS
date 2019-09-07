#!/usr/bin/env python

import threading
import rospy
from networktables import NetworkTables
from geometry_msgs.msg import Pose, Twist

IP = 'localhost'

ros_table = None

subscribed_topics = dict()

#Threading stuff for connecting to NT
cond = threading.Condition()
notified = [False]

def connectionListener(connected, info):
    print(info, '; Connected=%s' % connected)
    with cond:
        notified[0] = True
        cond.notify()

def add_to_nt(data, topic):
    ros_table.putString(topic, data)

def transmit():
    prev_command = ""
    while True:
        command_control = ros_table.getString("command_control", "")

        if command_control != prev_command:
            print("Command is: " + command_control)
            command_words = command_control.split()
            if command_words[0] == "subscribe":
                topic = rospy.Subscriber(command_words[1], Twist, add_to_nt, callback_args=command_words[1])
                subscribed_topics = {command_words[1] : topic}
                print(topic)
            if command_words[0] == "unsubscribe":
                print(subscribed_topics[command_words[1]])
                subscribed_topics[command_words[1]].unregister()

        prev_command = command_control

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
        #Get subtable to appear!
        ros_table.putNumber("test", 1)
        rospy.init_node('network_table_bridge', anonymous=False)


        #my_topic = 'frc_diff_drive_controller/cmd_vel'
        #rospy.Subscriber(my_topic, Twist, add_to_nt, callback_args=my_topic)

        transmit()
    except rospy.ROSInterruptException:
        pass