#!/usr/bin/env python

# -*- coding: utf-8 -*-
#
# Copyright (c) 2011, Antons Rebguns
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
# 
# Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
# 
# Neither the name of the <ORGANIZATION> nor the names of its contributors may
# be used to endorse or promote products derived from this software without
# specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import roslib; roslib.load_manifest('w2_object_manipulation_launch')
import rospy

from ua_audio_infomax.msg import Action as InfomaxAction
from ua_audio_infomax.srv import InfoMax
from ua_audio_infomax.srv import InfoMaxResponse
from ua_audio_capture.srv import classify

from wubble2_gripper_controller.msg import WubbleGripperAction
from wubble2_gripper_controller.msg import WubbleGripperGoal

from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus

from tabletop_object_detector.msg import TabletopDetectionResult

from tabletop_collision_map_processing.srv import TabletopCollisionMapProcessing
from tabletop_collision_map_processing.srv import TabletopCollisionMapProcessingRequest

from object_manipulation_msgs.msg import GraspHandPostureExecutionGoal
from object_manipulation_msgs.msg import GraspHandPostureExecutionAction
from object_manipulation_msgs.srv import GraspStatus

from arm_navigation_msgs.msg import OrderedCollisionOperations
from arm_navigation_msgs.msg import CollisionOperation

from object_detection import ObjectDetector

from w2_object_manipulation_launch.actions_common import ARM_GROUP_NAME
from w2_object_manipulation_launch.actions_common import GRIPPER_GROUP_NAME
from w2_object_manipulation_launch.actions_common import find_current_arm_state

from w2_object_manipulation_launch.msg import DropObjectAction
from w2_object_manipulation_launch.msg import GraspObjectAction
from w2_object_manipulation_launch.msg import LiftObjectAction
from w2_object_manipulation_launch.msg import PlaceObjectAction
from w2_object_manipulation_launch.msg import PushObjectAction
from w2_object_manipulation_launch.msg import ShakePitchObjectAction
from w2_object_manipulation_launch.msg import ShakeRollObjectAction
from w2_object_manipulation_launch.msg import ReadyArmAction

from w2_object_manipulation_launch.msg import DropObjectGoal
from w2_object_manipulation_launch.msg import GraspObjectGoal
from w2_object_manipulation_launch.msg import LiftObjectGoal
from w2_object_manipulation_launch.msg import PlaceObjectGoal
from w2_object_manipulation_launch.msg import PushObjectGoal
from w2_object_manipulation_launch.msg import ShakePitchObjectGoal
from w2_object_manipulation_launch.msg import ShakeRollObjectGoal
from w2_object_manipulation_launch.msg import ReadyArmGoal

class ObjectCategorizer():
    def __init__(self):
        self.object_detector = ObjectDetector()
        
        # connect to collision map processing service
        rospy.loginfo('waiting for tabletop_collision_map_processing service')
        rospy.wait_for_service('/tabletop_collision_map_processing/tabletop_collision_map_processing')
        self.collision_map_processing_srv = rospy.ServiceProxy('/tabletop_collision_map_processing/tabletop_collision_map_processing', TabletopCollisionMapProcessing)
        rospy.loginfo('connected to tabletop_collision_map_processing service')
        
        # connect to gripper action server
        rospy.loginfo('waiting for wubble_gripper_grasp_action')
        self.posture_controller = SimpleActionClient('/wubble_gripper_grasp_action', GraspHandPostureExecutionAction)
        self.posture_controller.wait_for_server()
        rospy.loginfo('connected to wubble_gripper_grasp_action')
        
        rospy.loginfo('waiting for wubble_grasp_status service')
        rospy.wait_for_service('/wubble_grasp_status')
        self.get_grasp_status_srv = rospy.ServiceProxy('/wubble_grasp_status', GraspStatus)
        rospy.loginfo('connected to wubble_grasp_status service')
        
        rospy.loginfo('waiting for classify service')
        rospy.wait_for_service('/classify')
        self.classification_srv = rospy.ServiceProxy('/classify', classify)
        rospy.loginfo('connected to classify service')
        
        # connect to gripper action server
        rospy.loginfo('waiting for wubble_gripper_action')
        self.gripper_controller = SimpleActionClient('/wubble_gripper_action', WubbleGripperAction)
        self.gripper_controller.wait_for_server()
        rospy.loginfo('connected to wubble_gripper_action')
        
        # connect to wubble actions
        rospy.loginfo('waiting for drop_object')
        self.drop_object_client = SimpleActionClient('/drop_object', DropObjectAction)
        self.drop_object_client.wait_for_server()
        rospy.loginfo('connected to drop_object')
        
        rospy.loginfo('waiting for grasp_object')
        self.grasp_object_client = SimpleActionClient('/grasp_object', GraspObjectAction)
        self.grasp_object_client.wait_for_server()
        rospy.loginfo('connected to grasp_object')
        
        rospy.loginfo('waiting for lift_object')
        self.lift_object_client = SimpleActionClient('/lift_object', LiftObjectAction)
        self.lift_object_client.wait_for_server()
        rospy.loginfo('connected to lift_object')
        
        rospy.loginfo('waiting for place_object')
        self.place_object_client = SimpleActionClient('/place_object', PlaceObjectAction)
        self.place_object_client.wait_for_server()
        rospy.loginfo('connected to plcae_object')
        
        rospy.loginfo('waiting for push_object')
        self.push_object_client = SimpleActionClient('/push_object', PushObjectAction)
        self.push_object_client.wait_for_server()
        rospy.loginfo('connected to push_object')
        
        rospy.loginfo('waiting for shake_roll_object')
        self.shake_roll_object_client = SimpleActionClient('/shake_roll_object', ShakeRollObjectAction)
        self.shake_roll_object_client.wait_for_server()
        rospy.loginfo('connected to drop_object')
        
        rospy.loginfo('waiting for shake_pitch_object')
        self.shake_pitch_object_client = SimpleActionClient('/shake_pitch_object', ShakePitchObjectAction)
        self.shake_pitch_object_client.wait_for_server()
        rospy.loginfo('connected to pitch_object')
        
        rospy.loginfo('waiting for ready_arm')
        self.ready_arm_client = SimpleActionClient('/ready_arm', ReadyArmAction)
        self.ready_arm_client.wait_for_server()
        rospy.loginfo('connected to ready_arm')
        
        # advertise InfoMax service
        rospy.Service('get_category_distribution', InfoMax, self.process_infomax_request)
        
        rospy.loginfo('all services contacted, object_categorization is ready to go')
        
        self.ACTION_INFO = {
            InfomaxAction.GRASP: {
                'client': self.grasp_object_client,
                'goal': GraspObjectGoal(),
                'prereqs': [InfomaxAction.GRASP]
            },
            
            InfomaxAction.LIFT: {
                'client': self.lift_object_client,
                'goal': LiftObjectGoal(),
                'prereqs': [InfomaxAction.GRASP, InfomaxAction.LIFT]
            },
            
            InfomaxAction.SHAKE_ROLL: {
                'client': self.shake_roll_object_client,
                'goal': ShakeRollObjectGoal(),
                'prereqs': [InfomaxAction.GRASP, InfomaxAction.LIFT, InfomaxAction.SHAKE_ROLL]
            },
            
            InfomaxAction.DROP: {
                'client': self.drop_object_client,
                'goal': DropObjectGoal(),
                'prereqs': [InfomaxAction.GRASP, InfomaxAction.LIFT, InfomaxAction.DROP]
            },
            
            InfomaxAction.PLACE: {
                'client': self.place_object_client,
                'goal': PlaceObjectGoal(),
                'prereqs': [InfomaxAction.GRASP, InfomaxAction.LIFT, InfomaxAction.PLACE]
            },
            
            InfomaxAction.PUSH: {
                'client': self.push_object_client,
                'goal': PushObjectGoal(),
                'prereqs': [InfomaxAction.GRASP, InfomaxAction.LIFT, InfomaxAction.PLACE, InfomaxAction.PUSH]
            },
            
            InfomaxAction.SHAKE_PITCH: {
                'client': self.shake_pitch_object_client,
                'goal': ShakePitchObjectGoal(),
                'prereqs': [InfomaxAction.GRASP, InfomaxAction.LIFT, InfomaxAction.SHAKE_PITCH]
            },
        }


    def open_gripper(self):
        pg = GraspHandPostureExecutionGoal()
        pg.goal = GraspHandPostureExecutionGoal.RELEASE
        
        self.posture_controller.send_goal(pg)
        self.posture_controller.wait_for_result()


    def gentle_close_gripper(self):
        pg = GraspHandPostureExecutionGoal()
        pg.goal = GraspHandPostureExecutionGoal.GRASP
        
        self.posture_controller.send_goal(pg)
        self.posture_controller.wait_for_result()
        
        goal = WubbleGripperGoal()
        goal.command = WubbleGripperGoal.CLOSE_GRIPPER
        goal.torque_limit = 0.0
        goal.dynamic_torque_control = False
        
        self.gripper_controller.send_goal(goal)
        self.gripper_controller.wait_for_result()


    def segment_objects(self):
        res = self.object_detector.detect()
        
        if res is None:
            rospy.logerr('TabletopSegmentation did not find any clusters')
            return None
            
        segmentation_result = res[0]
        self.info = res[1]
        
        tdr = TabletopDetectionResult()
        tdr.table = segmentation_result.table
        tdr.clusters = segmentation_result.clusters
        tdr.result = segmentation_result.result
        
        tcmpr = self.update_collision_map(tdr)
        
        return tcmpr


    def reset_collision_map(self):
        req = TabletopCollisionMapProcessingRequest()
        req.detection_result = TabletopDetectionResult()
        req.reset_collision_models = True
        req.reset_attached_models = True
        
        self.collision_map_processing_srv(req)
        rospy.loginfo('collision map reset')


    def update_collision_map(self, tabletop_detection_result):
        rospy.loginfo('collision_map update in progress')
        
        req = TabletopCollisionMapProcessingRequest()
        req.detection_result = tabletop_detection_result
        req.reset_collision_models = True
        req.reset_attached_models = True
        req.desired_frame = 'base_link'
        
        res = self.collision_map_processing_srv(req)
        
        rospy.loginfo('collision_map update is done')
        rospy.loginfo('there are %d graspable objects %s, collision support surface name is "%s"' %
                      (len(res.graspable_objects), res.collision_object_names, res.collision_support_surface_name))
                      
        return res


    def reset_robot(self, tabletop_collision_map_processing_result=None):
        rospy.loginfo('resetting robot')
        ordered_collision_operations = OrderedCollisionOperations()
        
        if tabletop_collision_map_processing_result:
            closest_index = self.info[0][0]
            collision_object_name = tabletop_collision_map_processing_result.collision_object_names[closest_index]
            collision_support_surface_name = tabletop_collision_map_processing_result.collision_support_surface_name
            
            collision_operation = CollisionOperation()
            collision_operation.object1 = collision_support_surface_name
            collision_operation.object2 = ARM_GROUP_NAME
            collision_operation.operation = CollisionOperation.DISABLE
            collision_operation.penetration_distance = 0.02
            ordered_collision_operations.collision_operations = [collision_operation]
            
            collision_operation = CollisionOperation()
            collision_operation.object1 = collision_support_surface_name
            collision_operation.object2 = GRIPPER_GROUP_NAME
            collision_operation.operation = CollisionOperation.DISABLE
            collision_operation.penetration_distance = 0.02
            ordered_collision_operations.collision_operations.append(collision_operation)
            
            collision_operation = CollisionOperation()
            collision_operation.object1 = collision_object_name
            collision_operation.object2 = GRIPPER_GROUP_NAME
            collision_operation.operation = CollisionOperation.DISABLE
            collision_operation.penetration_distance = 0.02
            ordered_collision_operations.collision_operations.append(collision_operation)
        else:
            self.reset_collision_map()
            
        current_state = find_current_arm_state()
        rospy.loginfo('arm is currently in %s state' % current_state)
        
        if current_state not in ['READY', 'TUCK']:
            # check if the gripper is empty
            grasp_status = self.get_grasp_status_srv()
            if grasp_status.is_hand_occupied:
                rospy.loginfo('arm is not in any of the following states %s, opening gripper' % str(['READY', 'TUCK']))
                self.open_gripper()
                
        if current_state != 'READY' and not self.ready_arm(ordered_collision_operations): return False
        self.gentle_close_gripper()
        return True


    def ready_arm(self, collision_operations=OrderedCollisionOperations()):
        """
        Moves the arm to the side out of the view of all sensors. In this
        position the arm is ready to perform a grasp action.
        """
        goal = ReadyArmGoal()
        goal.collision_operations = collision_operations
        state = self.ready_arm_client.send_goal_and_wait(goal)
        
        if state == GoalStatus.SUCCEEDED: return True
        else: return False


    def execute_action(self, action, tabletop_collision_map_processing_result):
        goal = self.ACTION_INFO[action]['goal']
        goal.category_id = self.category_id
        closest_index = self.info[0][0]
        goal.graspable_object = tabletop_collision_map_processing_result.graspable_objects[closest_index]
        goal.collision_object_name = tabletop_collision_map_processing_result.collision_object_names[closest_index]
        goal.collision_support_surface_name = tabletop_collision_map_processing_result.collision_support_surface_name
        state = self.ACTION_INFO[action]['client'].send_goal_and_wait(goal)
        
        if state == GoalStatus.SUCCEEDED:
            result = self.ACTION_INFO[action]['client'].get_result()
            return result.recorded_sound
        else:
            return None


    # receive an InfoMax service request containing object ID and desired action
    def process_infomax_request(self, req):
        self.object_names = req.objectNames
        self.action_names = req.actionNames
        self.num_categories = req.numCats
        self.category_id = req.catID
        
        if not self.reset_robot(): return None
        
        # find a graspable object on the floor
        tcmpr = self.segment_objects()
        if tcmpr is None: return None
        
        # initialize as uniform distribution
        beliefs = [1.0/self.num_categories] * self.num_categories
        actions = self.ACTION_INFO[req.actionID.val]['prereqs']
        
        for act in actions:
            sound = self.execute_action(act, tcmpr)
            
            if not sound:
                self.reset_robot(tcmpr)
                return None
            else:
                resp = self.classification_srv(sound)
                beliefs = resp.beliefs
                
        if not self.reset_robot(tcmpr): return None
        
        res = InfoMaxResponse()
        res.beliefs = beliefs
        res.location = self.category_id
        return res


if __name__ == '__main__':
    rospy.init_node('infomax_yippie_node', anonymous=True)
    oc = ObjectCategorizer()
    oc.reset_robot()
    rospy.spin()

