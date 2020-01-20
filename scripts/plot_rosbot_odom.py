#! /usr/bin/python

import rospy
import sys

from pa1.rosbot import Rosbot

if __name__ == "__main__":
    rospy.init_node("rosbot8_plot_odom", anonymous=False)

    rosbot = Rosbot()

    rospy.loginfo("Rosbot8 control initialized.")
    rosbot.plot_odom()
