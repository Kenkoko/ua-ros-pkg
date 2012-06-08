#!/usr/bin/env python

import roslib; roslib.load_manifest('w2_object_manipulation_launch')
import rospy

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

from wubble2_gripper_controller.msg import WubbleGripperAction
from wubble2_gripper_controller.msg import WubbleGripperGoal

from w2_object_manipulation_launch.msg import GraspObjectAction
from w2_object_manipulation_launch.msg import GraspObjectGoal

from w2_object_manipulation_launch.msg import LiftObjectAction
from w2_object_manipulation_launch.msg import LiftObjectGoal

from w2_object_manipulation_launch.msg import PlaceObjectAction
from w2_object_manipulation_launch.msg import PlaceObjectGoal

from w2_object_manipulation_launch.msg import ReadyArmAction
from w2_object_manipulation_launch.msg import ReadyArmGoal

from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point


class ResetRobot():
    def __init__(self):
        self.action_index = 0
        self.posture_controller = SimpleActionClient('/wubble_gripper_grasp_action', GraspHandPostureExecutionAction)
        self.gripper_controller = SimpleActionClient('/wubble_gripper_action', WubbleGripperAction)
        self.ready_arm_client = SimpleActionClient('/ready_arm', ReadyArmAction)
        self.collision_map_processing_srv = rospy.ServiceProxy('/tabletop_collision_map_processing/tabletop_collision_map_processing', TabletopCollisionMapProcessing)
        self.get_grasp_status_srv = rospy.ServiceProxy('/wubble_grasp_status', GraspStatus)

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
        
        return state == GoalStatus.SUCCEEDED

    def reset_collision_map(self):
        req = TabletopCollisionMapProcessingRequest()
        req.detection_result = TabletopDetectionResult()
        req.reset_collision_models = True
        req.reset_attached_models = True
        
        self.collision_map_processing_srv(req)
        rospy.loginfo('collision map reset')

    def execute(self):
        rospy.loginfo('resetting robot')
        self.action_index = 0
        self.reset_collision_map()
        
        current_state = find_current_arm_state()
        
        if current_state not in ['READY', 'TUCK']:
            grasp_status = self.get_grasp_status_srv()
            if grasp_status.is_hand_occupied: self.open_gripper()
            
        if current_state != 'READY' and not self.ready_arm(): return False
        self.gentle_close_gripper()
        return True


class SegmentScene():
    def __init__(self):
        self.object_detector = ObjectDetector()
        self.collision_map_processing_srv = rospy.ServiceProxy('/tabletop_collision_map_processing/tabletop_collision_map_processing',
                                                               TabletopCollisionMapProcessing)

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

    def execute(self):
        res = self.object_detector.detect()
        if res is None: return None
        
        segmentation_result = res['segmentation_result']
        
        tdr = TabletopDetectionResult()
        tdr.table = segmentation_result.table
        tdr.clusters = segmentation_result.clusters
        tdr.result = segmentation_result.result
        tcmpr = self.update_collision_map(tdr)
        
        return (tcmpr, res['cluster_information'])


if __name__ == '__main__':
    rospy.init_node('grab_and_run')
    
    reset = ResetRobot()
    reset.execute()
    
    segment = SegmentScene()
    res = segment.execute()
    if not res: exit(1)
    
    closest_index = res[1][0][0]
    grasp_client = SimpleActionClient('grasp_object', GraspObjectAction)
    grasp_client.wait_for_server()
    
    grasp_goal = GraspObjectGoal()
    grasp_goal.category_id = -1
    grasp_goal.graspable_object = res[0].graspable_objects[closest_index]
    grasp_goal.collision_object_name = res[0].collision_object_names[closest_index]
    grasp_goal.collision_support_surface_name = res[0].collision_support_surface_name
    
    result = grasp_client.send_goal_and_wait(grasp_goal)
    
    if result != GoalStatus.SUCCEEDED:
        rospy.logerr('failed to grasp')
        reset.execute()
        exit(1)
        
    lift_client = SimpleActionClient('lift_object', LiftObjectAction)
    lift_client.wait_for_server()
    
    lift_goal = LiftObjectGoal()
    lift_goal.category_id = 0
    lift_goal.graspable_object = res[0].graspable_objects[closest_index]
    lift_goal.collision_object_name = res[0].collision_object_names[closest_index]
    lift_goal.collision_support_surface_name = res[0].collision_support_surface_name
    
    result = lift_client.send_goal_and_wait(lift_goal)
    
    if result != GoalStatus.SUCCEEDED:
        rospy.logerr('failed to lift')
        exit(1)
        
    move_client = SimpleActionClient('/move_base', MoveBaseAction)
    move_client.wait_for_server()
    
    mbg = MoveBaseGoal()
    mbg.target_pose.header.stamp = rospy.Time.now()
    mbg.target_pose.header.frame_id = '/map'
    mbg.target_pose.pose.position = Point(0.896, -2.116, 0.000)
    mbg.target_pose.pose.orientation = Quaternion(0, 0, 0.350, 0.937)
    
    move_client.send_goal(mbg)
    move_client.wait_for_result()
    
    result = move_client.get_state()
    
    if result != GoalStatus.SUCCEEDED:
        rospy.logerr('failed to move')
        exit(1)
        
    place_client = SimpleActionClient('place_object', PlaceObjectAction)
    place_client.wait_for_server()
    
    place_goal = LiftObjectGoal()
    place_goal.category_id = 0
    place_goal.graspable_object = res[0].graspable_objects[closest_index]
    place_goal.collision_object_name = res[0].collision_object_names[closest_index]
    place_goal.collision_support_surface_name = res[0].collision_support_surface_name
    
    result = place_client.send_goal_and_wait(place_goal)
    
    if result != GoalStatus.SUCCEEDED:
        rospy.logerr('failed to place')
        exit(1)
        
    if not reset.execute():
        rospy.logerr('failed to reset robot')
        exit(1)
        
    mbg.target_pose.header.stamp = rospy.Time.now()
    mbg.target_pose.header.frame_id = '/map'
    mbg.target_pose.pose.position = Point(0.306, 0.305, 0.000)
    mbg.target_pose.pose.orientation = Quaternion(0, 0, 0, 1)
    
    move_client.send_goal(mbg)
    move_client.wait_for_result()
    
    result = move_client.get_state()
    
    if result != GoalStatus.SUCCEEDED:
        rospy.logerr('failed to move')
        exit(1)
        

