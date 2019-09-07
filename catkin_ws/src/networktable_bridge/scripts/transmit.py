#!/usr/bin/env python

import threading
import rospy
from networktables import NetworkTables

IP = 'localhost'

#Threading stuff for connecting to NT
cond = threading.Condition()
notified = [False]

def connectionListener(connected, info):
    print(info, '; Connected=%s' % connected)
    with cond:
        notified[0] = True
        cond.notify()

def transmit():
    pass

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
        transmit()
    except rospy.ROSInterruptException:
        pass