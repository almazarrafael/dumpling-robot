import rospy
from sensor_msgs.msg import Joy

from playsound import playsound

buttons = None

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
            playsound("./sounds/honk.mp3")
        elif buttons[1] == 1:
            playsound("./sounds/fart.mp3")
        elif buttons[2] == 1:
            playsound("./sounds/drive_toggle.wav")
        elif buttons[3] == 1:
            playsound("./sounds/fortnite_death.mp3")
        else:
            pass

    rospy.spin()

if __name__ == '__main__':
    listener()
