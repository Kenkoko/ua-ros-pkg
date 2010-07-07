#!/usr/bin/env python
import roslib; roslib.load_manifest('smart_arm_affordance')
import rospy
import time
import math
from std_msgs.msg import String
# ROS imports
from std_msgs.msg import Float64
from ua_controller_msgs.msg import JointState

rospy.init_node('affordance_listener', anonymous=True)
wrist = rospy.Publisher('/wrist_rotate_controller/command', Float64)
left_finger = rospy.Publisher('/finger_left_controller/command', Float64)
right_finger = rospy.Publisher('/finger_right_controller/command', Float64)
elbow = rospy.Publisher('/elbow_tilt_controller/command', Float64)
shoulder = rospy.Publisher('/shoulder_tilt_controller/command', Float64)
pan = rospy.Publisher('/shoulder_pan_controller/command', Float64)
time.sleep(1)

templeft = 0.0
tempright = 0.0

def squeeze(times, force, closed, opened):
    global templeft
    global tempright
    while templeft == 0:
        rospy.sleep(.1)

    tries = int(opened - closed)
    squeezeForce = 0
    print 'Force: ' + str(force)
    print 'Tries: ' + str(tries)
    print 'Left temp: ' + str(templeft)
    print 'Right temp: ' + str(tempright)
    for j in range(tries):
        print 'Left temp: ' + str(templeft)
        print 'Right temp: ' + str(tempright)
        print 'Force: ' + str(force)
        if (templeft >= force) | (tempright >= force):
            print 'Exceeded force threshold'
            squeezeForce = j
            break
        else:
            print j + 1
            left_finger.publish( math.radians(opened - j) )
            right_finger.publish( math.radians(-opened + j) )
            rospy.sleep(.3)
            j -= 5

    print 'outside squeezeForce: ' + str(squeezeForce)
#    for i in range(times):
#        time.sleep(1)
#        left_finger.publish( math.radians(opened) )
#        right_finger.publish( math.radians(-opened) )
#        print 'opened...'
#        time.sleep(1)
#        left_finger.publish( math.radians(closed) )
#        right_finger.publish( math.radians(-closed) )
#        print 'closed...'
#        print i + 1
    rospy.sleep(1)
    left_finger.publish( math.radians(squeezeForce) )
    right_finger.publish( math.radians(-squeezeForce) )

def cobra_pose():
    wrist.publish( math.radians(0.0) )
    left_finger.publish( math.radians(30.0) )
    right_finger.publish( math.radians(-30.0) )
    elbow.publish( math.radians(-90.0) )
    shoulder.publish( math.radians(90.0) )
    pan.publish( math.radians(90.0) )


def relaxed_cobra_pose():
    wrist.publish( math.radians(0.0) )
    left_finger.publish( math.radians(30.0) )
    right_finger.publish( math.radians(-30.0) )
    elbow.publish( math.radians(-135.0) )
    shoulder.publish( math.radians(135.0) )
    pan.publish( math.radians(90.0) )

def custom_pose(p, s, e, w, l, r):
    wrist.publish( math.radians(w) )
    left_finger.publish( math.radians(l) )
    right_finger.publish( math.radians(r) )
    elbow.publish( math.radians(e) )
    shoulder.publish( math.radians(s) )
    pan.publish( math.radians(p) )

def callLeft(msg):
    print 'Left claw load: ' + str(msg.load) + '\n'
    global templeft
    templeft = msg.load

def callRight(msg):
    print 'Right claw load: ' + str(msg.load) + '\n'
    global tempright
    tempright = msg.load

if __name__ == '__main__':
    global templeft
    global tempright
    rospy.Subscriber("/finger_left_controller/state", JointState, callLeft)
    rospy.Subscriber("/finger_right_controller/state", JointState, callRight)

#    while not rospy.is_shutdown():
#        print 'Left: ' + str(templeft)
#        print 'Right: ' + str(tempright)
#        rospy.sleep(1)

#    cobra_pose()
#    time.sleep(1)
#    relaxed_cobra_pose()
#    time.sleep(1)

#    for i in range(5):
##        print getLeft()
#        print templeft

    squeeze(10, 250, -5.0, 5.0)
    rospy.sleep(1)
    relaxed_cobra_pose()
#    custom_pose(90.0, 0.0, 0.0, 0.0, 30.0, -30.0)
#    time.sleep(1)
    #relaxed_cobra_pose()
