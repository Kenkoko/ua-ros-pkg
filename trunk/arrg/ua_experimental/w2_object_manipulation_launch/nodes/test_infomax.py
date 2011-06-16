#!/usr/bin/env python

# Author: Antons Rebguns

import roslib; roslib.load_manifest('w2_object_manipulation_launch')
import rospy

from ua_audio_infomax.msg import Action as InfomaxAction
from ua_audio_infomax.srv import InfoMax
from ua_audio_infomax.srv import InfoMaxRequest
from ua_audio_capture.srv import classify

def process_classify(req):
    return (['obj1','obj2'], [0.5,0.5])

if __name__ == '__main__':
    rospy.Service('/classify', classify, process_classify)

    rospy.init_node('infomax_test', anonymous=True)
    rospy.loginfo('waiting for /get_category_distribution service')
    rospy.wait_for_service('/get_category_distribution')
    infomax_srv = rospy.ServiceProxy('/get_category_distribution', InfoMax)
    rospy.loginfo('connected to get_category_distribution service')
    
    actions = [InfomaxAction.SHAKE_ROLL]*1
    
    req = InfoMaxRequest()
    req.objectNames = ['pink_glass',        #0
                       'german_ball',       #1
                       'blue_cup',          #2
                       'blue_spiky_ball',   #3
                       'screw_box',         #4
                       'wire_spool',        #5
                       'sqeaky_ball',       #6
                       'duck_tape_roll',    #7
                       'ace_terminals',     #8
                       'chalkboard_eraser', #9
                       ]
    req.actionNames = map(str, actions)
    req.numCats = len(req.objectNames)
    req.catID = 4

    for trial in range(1):
        rospy.loginfo('Trial number %d, object is %s' % (trial,req.objectNames[req.catID]))
        
        for action in actions:
            req.actionID.val = action
            
            try:
                infomax_srv(req)
                rospy.loginfo('infomax test done action %d' % action)
            except Exception as e:
                rospy.logerr('Something happened: %s' % str(e))


