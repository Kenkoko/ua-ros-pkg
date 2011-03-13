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
    
    actions = [InfomaxAction.GRASP, InfomaxAction.LIFT, InfomaxAction.DROP, InfomaxAction.SHAKE_ROLL, InfomaxAction.PLACE]
    
    req = InfoMaxRequest()
    req.objectNames = ['obj1', 'obj2', 'obj3']
    req.actionNames = map(str, actions)
    req.numCats = len(req.objectNames)
    req.catID = 2


    for action in actions:
        req.actionID.val = action
        
        infomax_srv(req)
        rospy.loginfo('infomax test done action %d' % action)


