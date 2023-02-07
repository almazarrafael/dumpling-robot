#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Joy

from playsound import playsound

from pathlib import Path

SCRIPT_DIR = Path(__file__).parent

buttons = None

path = "~/dumpling-robot/ros_ws/src/joy2sound/src"

def callback(data):
    global buttons
    buttons = data.buttons

def listener():
    global buttons
    rospy.init_node("joy2sound", anonymous=True)
    rospy.Subscriber("joy", Joy, callback)

    rate = rospy.Rate(60)

    while not rospy.is_shutdown():
        if buttons == None:
            pass
        elif buttons[0] == 1:
            playsound(SCRIPT_DIR / 'sounds/honk.mp3')
        elif buttons[1] == 1:
            playsound(SCRIPT_DIR / "sounds/fart.mp3")
        elif buttons[2] == 1 or buttons[4] == 1:
            playsound(SCRIPT_DIR / "sounds/drive_toggle.wav")
        elif buttons[3] == 1:
            playsound(SCRIPT_DIR / "sounds/fortnite_death.mp3")
        else:
            pass
        rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    listener()
