#!/usr/bin/env python

from threading import Thread

import roslib; roslib.load_manifest('w2_object_manipulation_launch')
import rospy

import smach
from smach import State
from smach import CBState
from smach import StateMachine
from smach_ros import SimpleActionState
from smach_ros import IntrospectionServer

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
from tabletop_collision_map_processing.srv import TabletopCollisionMapProcessingResponse

from object_manipulation_msgs.msg import GraspHandPostureExecutionGoal
from object_manipulation_msgs.msg import GraspHandPostureExecutionAction
from object_manipulation_msgs.srv import GraspStatus

from arm_navigation_msgs.msg import OrderedCollisionOperations
from arm_navigation_msgs.msg import CollisionOperation

from w2_object_manipulation_launch.object_detection import ObjectDetector

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

from w2_object_manipulation_launch.msg import DoInfomaxAction
from w2_object_manipulation_launch.msg import DoInfomaxFeedback
from w2_object_manipulation_launch.msg import DoInfomaxResult

from point_cloud_classifier.srv import GetClusterLabels
from point_cloud_classifier.srv import GetClusterLabelsRequest

from smach_ros import ActionServerWrapper

class ResetRobot(State):
    def __init__(self):
        self.posture_controller = SimpleActionClient('/wubble_gripper_grasp_action', GraspHandPostureExecutionAction)
        self.gripper_controller = SimpleActionClient('/wubble_gripper_action', WubbleGripperAction)
        self.ready_arm_client = SimpleActionClient('/ready_arm', ReadyArmAction)
        self.collision_map_processing_srv = rospy.ServiceProxy('/tabletop_collision_map_processing/tabletop_collision_map_processing', TabletopCollisionMapProcessing)
        self.get_grasp_status_srv = rospy.ServiceProxy('/wubble_grasp_status', GraspStatus)
        
        State.__init__(self, output_keys=['action_index'], outcomes=['succeeded','aborted'])

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

    def reset_collision_map(self):
        req = TabletopCollisionMapProcessingRequest()
        req.detection_result = TabletopDetectionResult()
        req.reset_collision_models = True
        req.reset_attached_models = True
        
        self.collision_map_processing_srv(req)
        rospy.loginfo('collision map reset')

    def execute(self, userdata):
        rospy.loginfo('resetting robot')
        userdata.action_index = 0
        self.reset_collision_map()
        
        current_state = find_current_arm_state()
        
        if current_state not in ['READY', 'TUCK']:
            grasp_status = self.get_grasp_status_srv()
            if grasp_status.is_hand_occupied: self.open_gripper()
            
        if current_state != 'READY' and not self.ready_arm(): return 'aborted'
        self.gentle_close_gripper()
        return 'succeeded'


class SegmentScene(State):
    def __init__(self):
        self.object_detector = ObjectDetector()
        self.collision_map_processing_srv = rospy.ServiceProxy('/tabletop_collision_map_processing/tabletop_collision_map_processing',
                                                               TabletopCollisionMapProcessing)
        State.__init__(self, 
                       outcomes=['found_clusters','no_clusters'],
                       output_keys=['segmentation_result','cluster_information'])

    def update_collision_map(self, tabletop_detection_result):
        req = TabletopCollisionMapProcessingRequest()
        req.detection_result = tabletop_detection_result
        req.reset_collision_models = True
        req.reset_attached_models = True
        req.desired_frame = 'base_link'
        
        res = self.collision_map_processing_srv(req)
        rospy.loginfo('found %d clusters: %s, surface name: %s' %
                      (len(res.graspable_objects), res.collision_object_names, res.collision_support_surface_name))
                      
        return res

    def execute(self, userdata):
        res = self.object_detector.detect()
        if res is None: return 'no_clusters'
        
        segmentation_result = res['segmentation_result']
        
        tdr = TabletopDetectionResult()
        tdr.table = segmentation_result.table
        tdr.clusters = segmentation_result.clusters
        tdr.result = segmentation_result.result
        tcmpr = self.update_collision_map(tdr)
        
        userdata.segmentation_result = tcmpr
        userdata.cluster_information = res['cluster_information']
        
        return 'found_clusters'


def create_find_clusters_sm():
    sm = StateMachine(outcomes=['succeeded','aborted'],
                      output_keys=['segmentation_result','cluster_information','action_index'])
                      
    with sm:
        StateMachine.add('RESET_ROBOT',
                         ResetRobot(),
                         remapping={'action_index':'action_index'},
                         transitions={'succeeded':'SEGMENT_SCENE'})
                          
        StateMachine.add('SEGMENT_SCENE',
                         SegmentScene(),
                         remapping={'segmentation_result':'segmentation_result',
                                    'cluster_information':'cluster_information'},
                         transitions={'found_clusters':'succeeded',
                                      'no_clusters':'aborted'})
                                      
    return sm


def create_grasp_sm():
    sm = StateMachine(outcomes=['succeeded','aborted','preempted'],
                      input_keys=['closest_index','segmentation_result'])
                      
    with sm:
        def grasp_goal_cb(userdata, goal):
            grasp_goal = GraspObjectGoal()
            grasp_goal.category_id = -1
            grasp_goal.graspable_object = userdata.segmentation_result.graspable_objects[userdata.closest_index]
            grasp_goal.collision_object_name = userdata.segmentation_result.collision_object_names[userdata.closest_index]
            grasp_goal.collision_support_surface_name = userdata.segmentation_result.collision_support_surface_name
            return grasp_goal
            
        StateMachine.add('GRASP_OBJECT',
                         SimpleActionState('grasp_object',
                                            GraspObjectAction,
                                            goal_cb=grasp_goal_cb,
                                            input_keys=['closest_index',
                                                        'segmentation_result']),
                         remapping={'closest_index':'closest_index',
                                    'segmentation_result':'segmentation_result'})
                                    
    return sm


class SelectNextAction(State):
    def __init__(self):
        self.policy = [
            #InfomaxAction.GRASP,
            #InfomaxAction.LIFT,
            #InfomaxAction.SHAKE_ROLL,
            #InfomaxAction.SHAKE_PITCH,
            InfomaxAction.DROP,
        ]
        
        self.action_to_outcome = {
            InfomaxAction.GRASP: 'grasp',
            InfomaxAction.LIFT: 'lift',
            InfomaxAction.SHAKE_ROLL: 'shake_roll',
            InfomaxAction.SHAKE_PITCH: 'shake_pitch',
            InfomaxAction.DROP: 'drop',
        }
        
        State.__init__(self, 
                       outcomes=['grasp',
                                 'lift',
                                 'shake_roll',
                                 'shake_pitch',
                                 'drop',
                                 'aborted'],
                       input_keys=['action_index_in'],
                       output_keys=['infomax_action_id',
                                    'action_index_out'])
                       
    def execute(self, userdata):
        if userdata.action_index_in < len(self.policy):
            next_action = self.policy[userdata.action_index_in]
            userdata.infomax_action_id = next_action
            userdata.action_index_out = userdata.action_index_in + 1
            return self.action_to_outcome[next_action]
        else:
            userdata.infomax_action_id = -1
            return 'aborted'
            
    def reset_state(self):
        self.current_action = 0


def recognize_objects(graspable_objects):
    req = GetClusterLabelsRequest()
    req.clusters = [go.cluster for go in graspable_objects]
    rospy.loginfo('waiting for /get_cluster_labels')
    rospy.wait_for_service('/get_cluster_labels')
    rospy.loginfo('done')
    rec_srv = rospy.ServiceProxy('/get_cluster_labels', GetClusterLabels)
    rospy.loginfo('created service proxy')
    res = rec_srv(req)
    rospy.loginfo('got result')
    return res.labels


if __name__ == '__main__':
    rospy.init_node('smach_example')
    
    # Create and start the introspection server (to be able to display the state machine using
    #  rosrun smach_viewer smach_viewer.py)
    infomax_sm = StateMachine(outcomes=['succeeded','aborted','preempted'],
                              input_keys=['do_infomax_goal'],
                              output_keys=['do_infomax_result','do_infomax_feedback'])
    infomax_sm.userdata.sm_action_index = 0
    infomax_sm.userdata.do_infomax_feedback = DoInfomaxFeedback()
    infomax_sm.userdata.do_infomax_result = DoInfomaxResult()
    
    sis = IntrospectionServer('infomax_machine', infomax_sm, '/INFOMAX_MACHINE')
    t = Thread(target=sis.start)
    t.start()
    
    with infomax_sm:
        find_clusters_sm = create_find_clusters_sm()
        StateMachine.add('FIND_CLUSTERS',
                         find_clusters_sm,
                         transitions={'succeeded':'SELECT_CLUSTER',
                                      'aborted':'FAIL_RESET_ROBOT'},
                         remapping={'segmentation_result':'segmentation_result',
                                    'cluster_information':'cluster_information',
                                    'action_index':'sm_action_index'})
                                    
        @smach.cb_interface(input_keys=['segmentation_result', 'cluster_information'],
                            output_keys=['closest_index'],
                            outcomes=['succeeded'])
        def select_object_from_clusters(userdata):
            labels = recognize_objects(userdata.segmentation_result.graspable_objects)
            print labels
            userdata.closest_index = userdata.cluster_information[0][0]
            return 'succeeded'
            
        StateMachine.add('SELECT_CLUSTER',
                         CBState(select_object_from_clusters),
                         remapping={'closest_index':'closest_index'},
                         transitions={'succeeded':'SELECT_NEXT_ACTION'})
                         
        grasp_sm = create_grasp_sm()
        StateMachine.add('GRASP_OBJECT',
                         grasp_sm,
                         transitions={'succeeded':'SELECT_NEXT_ACTION',
                                      'aborted':'FAIL_RESET_ROBOT'})
                                      
        def lift_goal_cb(userdata, goal):
            lift_goal = LiftObjectGoal()
            lift_goal.category_id = 0
            lift_goal.graspable_object = userdata.segmentation_result.graspable_objects[userdata.closest_index]
            lift_goal.collision_object_name = userdata.segmentation_result.collision_object_names[userdata.closest_index]
            lift_goal.collision_support_surface_name = userdata.segmentation_result.collision_support_surface_name
            return lift_goal
            
        StateMachine.add('LIFT_OBJECT',
                         SimpleActionState('lift_object',
                                            LiftObjectAction,
                                            goal_cb=lift_goal_cb,
                                            input_keys=['closest_index',
                                                        'segmentation_result']),
                         remapping={'closest_index':'closest_index',
                                    'segmentation_result':'segmentation_result'},
                         transitions={'aborted':'FAIL_RESET_ROBOT',
                                      'succeeded':'SELECT_NEXT_ACTION'})
                                      
        def shake_roll_goal_cb(userdata, goal):
            shake_goal = ShakeRollObjectGoal()
            shake_goal.category_id = 0
            shake_goal.graspable_object = userdata.segmentation_result.graspable_objects[userdata.closest_index]
            shake_goal.collision_object_name = userdata.segmentation_result.collision_object_names[userdata.closest_index]
            shake_goal.collision_support_surface_name = userdata.segmentation_result.collision_support_surface_name
            return shake_goal
            
        StateMachine.add('SHAKE_ROLL_OBJECT',
                         SimpleActionState('shake_roll_object',
                                            ShakeRollObjectAction,
                                            goal_cb=shake_roll_goal_cb,
                                            input_keys=['closest_index',
                                                        'segmentation_result']),
                         remapping={'closest_index':'closest_index',
                                    'segmentation_result':'segmentation_result'},
                         transitions={'aborted':'FAIL_RESET_ROBOT',
                                      'succeeded':'SELECT_NEXT_ACTION'})
                                      
        def shake_pitch_goal_cb(userdata, goal):
            shake_goal = ShakePitchObjectGoal()
            shake_goal.category_id = 0
            shake_goal.graspable_object = userdata.segmentation_result.graspable_objects[userdata.closest_index]
            shake_goal.collision_object_name = userdata.segmentation_result.collision_object_names[userdata.closest_index]
            shake_goal.collision_support_surface_name = userdata.segmentation_result.collision_support_surface_name
            return shake_goal
            
        StateMachine.add('SHAKE_PITCH_OBJECT',
                         SimpleActionState('shake_pitch_object',
                                            ShakePitchObjectAction,
                                            goal_cb=shake_pitch_goal_cb,
                                            input_keys=['closest_index',
                                                        'segmentation_result']),
                         remapping={'closest_index':'closest_index',
                                    'segmentation_result':'segmentation_result'},
                         transitions={'aborted':'FAIL_RESET_ROBOT',
                                      'succeeded':'SELECT_NEXT_ACTION'})
                                      
        def drop_goal_cb(userdata, goal):
            drop_goal = DropObjectGoal()
            drop_goal.category_id = 0
            drop_goal.graspable_object = userdata.segmentation_result.graspable_objects[userdata.closest_index]
            drop_goal.collision_object_name = userdata.segmentation_result.collision_object_names[userdata.closest_index]
            drop_goal.collision_support_surface_name = userdata.segmentation_result.collision_support_surface_name
            return drop_goal
            
        StateMachine.add('DROP_OBJECT',
                         SimpleActionState('drop_object',
                                            DropObjectAction,
                                            goal_cb=drop_goal_cb,
                                            input_keys=['closest_index',
                                                        'segmentation_result']),
                         remapping={'closest_index':'closest_index',
                                    'segmentation_result':'segmentation_result'},
                         transitions={'aborted':'FAIL_RESET_ROBOT',
                                      'succeeded':'SELECT_NEXT_ACTION'})
                                      
        StateMachine.add('SELECT_NEXT_ACTION',
                         SelectNextAction(),
                         remapping={'infomax_action_id':'infomax_action_id',
                                    'action_index_in':'sm_action_index',
                                    'action_index_out':'sm_action_index'},
                         transitions={'aborted':'succeeded',
                                      'grasp':'GRASP_OBJECT',
                                      'lift':'LIFT_OBJECT',
                                      'shake_roll':'SHAKE_ROLL_OBJECT',
                                      'shake_pitch':'SHAKE_PITCH_OBJECT',
                                      'drop':'DROP_OBJECT',
                                      })
                                      
        StateMachine.add('FAIL_RESET_ROBOT',
                         ResetRobot(),
                         remapping={'action_index':'sm_action_index'},
                         transitions={'succeeded':'aborted'})
        
    #outcome = infomax_sm.execute()
    #rospy.spin()
    #sis.stop()
    
    asw = ActionServerWrapper(
        'do_infomax',
        DoInfomaxAction,
        wrapped_container=infomax_sm,
        succeeded_outcomes=['succeeded'],
        aborted_outcomes=['aborted'],
        preempted_outcomes=['preempted'],
        goal_key='do_infomax_goal',
        feedback_key='do_infomax_feedback',
        result_key='do_infomax_result',
    )
    
    asw.run_server()
    sis.stop()
    t.join()

