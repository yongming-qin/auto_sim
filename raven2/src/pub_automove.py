#!/usr/bin/env python
""" pub_automove.py
Publishes raven_automove msgs over ros.
"""

__author__ = "Kyle Lindgren"
__version__ = "0.0.0"
__status__ = "Prototype"
__date__ = "Jan 19, 2018"

import rospy
from raven_2.msg import raven_automove
from geometry_msgs.msg import Quaternion, Vector3
from math import sin, cos

class Controller():
    """
    Class to hold raven controller.
    """
    def __init__(self):
        self.count = 0
        self.rate = 0.1

    def inc_count(self):
        self.count = self.count + self.rate

def raven_automove_msg_gen(Counter):
    pub = rospy.Publisher('/raven_automove', raven_automove, queue_size=10)
    rospy.init_node('raven_automove_msg_gen', anonymous=True)
    rate = rospy.Rate(500)
    msg = raven_automove()
    while not rospy.is_shutdown():
        #msg.tf_incr[0].translation = Vector3(2*cos(Counter.count/1000),
                                             #2*sin(Counter.count/1000), 2*sin(Counter.count/1000))
        msg.tf_incr[0].translation = Vector3(0, 0, 0)
        msg.tf_incr[1].translation = Vector3(0, 0, 0)
        # msg.tf_incr[0].translation = Vector3(1.0, 1.0, 1.0)
        msg.tf_incr[0].rotation = Quaternion(0, 0, 0, 1)
        msg.tf_incr[1].rotation = Quaternion(0, 0, 0, 1)

        msg.del_pos[0] = 0 # -1 open
        msg.del_pos[1] = 1

        Counter.inc_count()
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    Counter = Controller()
    try:
        raven_automove_msg_gen(Counter)
    except rospy.ROSInterruptException:
        pass
