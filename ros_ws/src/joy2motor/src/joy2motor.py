#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Joy

from adafruit_motorkit import MotorKit
kit = MotorKit()

import math

x = 0
y = 0
turn = 0

ry = 0

driveToggle = 0 # 0 = holonomic. 1 = tank. 2 = car

xBtnPrevState = 0

minThrottle = 0

lt = 0
rt = 0

breakBtn = 0

def callback(data):
    global x, y, turn, ry, driveToggle, xBtnPrevState, lt, rt, breakBtn
    x = -data.axes[0] # left stick x
    y = data.axes[1] # left stick y
    turn = -data.axes[3] # right stick x
    
    ry = data.axes[4]

    lt = 1 - (data.axes[2]+1)/2 # normalize to 0 to 1
    rt = 1 - (data.axes[5]+1)/2 # normalize to 0 to 1

    breakBtn = data.buttons[5]

    if xBtnPrevState == 1 and data.buttons[0] == 0:
        driveToggle = driveToggle + 1
        if driveToggle == 3:
            driveToggle = 0
    xBtnPrevState = data.buttons[2]

def motor(x, y, turn, ry, lt, rt, breakBtn):
    global driveToggle
    rospy.loginfo("x: %s", x)
    rospy.loginfo("y: %s", y)
    rospy.loginfo("turn: %s", turn)

    theta = math.atan2(y, x)
    power = math.hypot(x, y)

    sin = math.sin(theta - math.pi/4)
    cos = math.cos(theta - math.pi/4)
    maxVal = max(abs(sin), abs(cos))

    leftFront = power * cos/maxVal + turn
    rightFront = power * sin/maxVal - turn
    leftRear = power * sin/maxVal + turn
    rightRear = power * cos/maxVal - turn

    try:
        if ((power + abs(turn)) > 1):
            leftFront /= power + turn;
            rightFront /= power + turn;
            leftRear /= power + turn;
            rightRear /= power + turn;
    except:
        return

    leftFront = 1 if leftFront > 1 else leftFront
    leftFront = -1 if leftFront < -1 else leftFront

    rightFront = 1 if rightFront > 1 else rightFront
    rightFront = -1 if rightFront < -1 else rightFront

    leftRear = 1 if leftRear > 1 else leftRear
    leftRear = -1 if leftRear < -1 else leftRear

    rightRear = 1 if rightRear > 1 else rightRear
    rightRear = -1 if rightRear < -1 else rightRear

    rospy.loginfo(rospy.get_caller_id() + "Left_Front:[%s], Right_Front:[%s], Left_Rear:[%s], Right_Rear:[%s]", leftFront, rightFront, leftRear, rightRear)

    if driveToggle == 0:
        kit.motor3.throttle = leftFront
        kit.motor4.throttle = -rightFront
        kit.motor1.throttle = leftRear
        kit.motor2.throttle = rightRear
    elif driveToggle == 1:
        kit.motor3.throttle = y
        kit.motor1.throttle = y
        kit.motor4.throttle = -ry
        kit.motor2.throttle = ry
    elif driveToggle == 2:
        throttle = rt if lt < rt else -lt
        steer = turn
        rospy.loginfo("Steer: %s", steer)
        if breakBtn == 1:
            left = 0
            right = 0
        elif throttle == 0:
            left = None
            right = None
        else:
            if steer > 0: # turning right
                left = throttle
                right = throttle * (-(1-minThrottle)*steer + 1)
            elif steer < 0: # turning left
                left = throttle * ((1-minThrottle)*steer + 1)
                right = throttle
            else:
                left = throttle
                right = throttle
        kit.motor3.throttle = left
        kit.motor1.throttle = left
        try:
            kit.motor4.throttle = -right
        except:
            kit.motor4.throttle = None
        kit.motor2.throttle = right
    else: # I dont think this will ever be executed..
        pass 

def listener():
    rospy.init_node("joy2motor", anonymous=True)
    rospy.Subscriber("joy", Joy, callback)

    rate = rospy.Rate(60)

    while not rospy.is_shutdown():
        motor(x, y, turn, ry, lt, rt, breakBtn)
        rate.sleep()

    # rospy.spin()

if __name__ == '__main__':
    listener()
