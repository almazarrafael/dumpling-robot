#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Joy

from adafruit_motorkit import MotorKit
kit = MotorKit()

import math

# Left Stick
lx = 0
ly = 0

# Right Stick
rx = 0
ry = 0

# Triggers
lt = 0
rt = 0

# Others
breakBtn = 0 # Right bumper
turboBtn = 0 # Left bumper
turboToggle = False
driveToggle = 0 # Triangle: 0 = holonomic. 1 = tank. 2 = car
xBtnPrevState = 0

# Motor parameters
minThrottle = 0
maxThrottle = 0.5

def callback(data):
    global lx, ly, rx, ry, driveToggle, xBtnPrevState, lt, rt, breakBtn, turboToggle
    lx = -data.axes[0]
    ly = data.axes[1]

    rx = -data.axes[3] 
    ry = data.axes[4]

    if (data.buttons[6] == 1):
        lt = 1 - (data.axes[2]+1)/2 # normalize to 0 to 1
    else:
        lt = 0
    if (data.buttons[7] == 1):
        rt = 1 - (data.axes[5]+1)/2 # normalize to 0 to 1
    else:
        rt = 0

    breakBtn = data.buttons[5]

    if xBtnPrevState == 1 and data.buttons[0] == 0:
        driveToggle = driveToggle + 1
        if driveToggle == 3:
            driveToggle = 0
    xBtnPrevState = data.buttons[2]

    turboBtn = data.buttons[4]
    
    turboToggle = not turboToggle if turboBtn == 1 else turboToggle
    #rospy.loginfo("Turbo Toggle: %s", turboToggle)
    #maxThrottle = 1 if turboToggle == True else 0.5
    

def holonomicDrive(lx, ly, rx):
    global driveToggle

    theta = math.atan2(ly, lx)
    power = math.hypot(lx, ly)

    sin = math.sin(theta - math.pi/4)
    cos = math.cos(theta - math.pi/4)
    maxVal = max(abs(sin), abs(cos))

    leftFront = power * cos/maxVal + rx
    rightFront = power * sin/maxVal - rx
    leftRear = power * sin/maxVal + rx
    rightRear = power * cos/maxVal - rx

    try:
        if ((power + abs(rx)) > 1):
            leftFront /= power + rx
            rightFront /= power + rx
            leftRear /= power + rx
            rightRear /= power + rx
    except:
        rerx

    # Clamps values to -1 to 1
    leftFront = 1 if leftFront > 1 else leftFront
    leftFront = -1 if leftFront < -1 else leftFront

    rightFront = 1 if rightFront > 1 else rightFront
    rightFront = -1 if rightFront < -1 else rightFront

    leftRear = 1 if leftRear > 1 else leftRear
    leftRear = -1 if leftRear < -1 else leftRear

    rightRear = 1 if rightRear > 1 else rightRear
    rightRear = -1 if rightRear < -1 else rightRear

    # Apply power values to motors
    kit.motor3.throttle = leftFront * maxThrottle
    kit.motor4.throttle = -rightFront * maxThrottle
    kit.motor1.throttle = leftRear * maxThrottle
    kit.motor2.throttle = rightRear * maxThrottle

def tankDrive(ly, ry):
    global driveToggle

    kit.motor3.throttle = ly * maxThrottle
    kit.motor1.throttle = ly * maxThrottle
    kit.motor4.throttle = -ry * maxThrottle
    kit.motor2.throttle = ry * maxThrottle

def carDrive(rx, rt, lt, breakBtn):
    global driveToggle

    throttle = rt if lt < rt else -lt
    steer = rx
    if breakBtn == 1:
        left = 0
        right = 0
    elif throttle == 0:
        left = None
        right = None
    else:
        if steer > 0: # rxing right
            left = throttle * maxThrottle
            right = throttle * (-steer + 1) * maxThrottle
        elif steer < 0: # rxing left
            left = throttle * (steer + 1) * maxThrottle
            right = throttle * maxThrottle
        else:
            left = throttle * maxThrottle
            right = throttle * maxThrottle
    kit.motor3.throttle = left
    kit.motor1.throttle = left
    try:
        kit.motor4.throttle = -right
    except:
        kit.motor4.throttle = None
    kit.motor2.throttle = right

def listener():
    rospy.init_node("joy2motor", anonymous=True)
    rospy.Subscriber("joy", Joy, callback)

    rate = rospy.Rate(60)

    while not rospy.is_shutdown():
        maxThrottle = 1 if turboToggle == True else 0.5
        if driveToggle == 0:
            holonomicDrive(lx, ly, rx)
        elif driveToggle == 1:
            tankDrive(ly, ry)
        elif driveToggle == 2:
            carDrive(rx, rt, lt, breakBtn)
        else:
            pass 
        #rospy.loginfo("Max throttle: %s", maxThrottle)
        rate.sleep()

if __name__ == '__main__':
    listener()
