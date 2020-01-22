#! /usr/bin/python

import rospy
import sys

from pa1.rosbot import Rosbot

if __name__ == "__main__":

    distance = float(sys.argv[1])
    linear_speed = float(sys.argv[2])

    rospy.init_node("rosbot8_move_forward", anonymous=False)

    rosbot = Rosbot(distance, linear_speed)

    rospy.loginfo("Rosbot8 control initialized.")

    # moves forward at a given distance with the provided linear speed
    rosbot.move_forward()
