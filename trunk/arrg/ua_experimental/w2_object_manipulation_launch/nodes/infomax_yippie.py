#!/usr/bin/env python

# Author: Antons Rebguns

import math
import numpy as np

import roslib; roslib.load_manifest('w2_object_manipulation_launch')
import rospy

from rospy.exceptions import ROSException

from ua_audio_infomax.msg import Action as InfomaxAction
from ua_audio_infomax.srv import InfoMax
from ua_audio_infomax.srv import InfoMaxResponse
from ua_audio_capture.srv import StartAudioRecording
from ua_audio_capture.srv import StopAudioRecording
from ua_audio_capture.srv import classify
from dynamixel_msgs.msg import JointState as DynamixelJointState
from dynamixel_controllers.srv import SetSpeed
from wubble2_robot.msg import WubbleGripperAction
from wubble2_robot.msg import WubbleGripperGoal

from pr2_controllers_msgs.msg import JointTrajectoryControllerState
from pr2_controllers_msgs.msg import JointTrajectoryAction
from pr2_controllers_msgs.msg import JointTrajectoryGoal

from tf import TransformListener

from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus

from tabletop_object_detector.srv import TabletopSegmentation
from tabletop_object_detector.srv import TabletopSegmentationResponse
from tabletop_object_detector.msg import TabletopDetectionResult

from tabletop_collision_map_processing.srv import TabletopCollisionMapProcessing
from tabletop_collision_map_processing.srv import TabletopCollisionMapProcessingRequest

from object_manipulation_msgs.msg import PickupAction
from object_manipulation_msgs.msg import PickupActionGoal
from object_manipulation_msgs.msg import PickupGoal
from object_manipulation_msgs.msg import GraspableObject
from object_manipulation_msgs.msg import GraspPlanningErrorCode
from object_manipulation_msgs.msg import GraspHandPostureExecutionGoal
from object_manipulation_msgs.msg import GraspHandPostureExecutionAction
from object_manipulation_msgs.srv import GraspPlanning
from object_manipulation_msgs.srv import GraspPlanningRequest
from object_manipulation_msgs.srv import GraspStatus

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from geometry_msgs.msg import Point32
from sensor_msgs.msg import JointState

from kinematics_msgs.srv import GetKinematicSolverInfo
from kinematics_msgs.srv import GetPositionIK
from kinematics_msgs.srv import GetPositionIKRequest
from kinematics_msgs.srv import GetPositionFK
from kinematics_msgs.srv import GetPositionFKRequest

from motion_planning_msgs.msg import PositionConstraint
from motion_planning_msgs.msg import OrientationConstraint
from motion_planning_msgs.msg import JointConstraint
from motion_planning_msgs.msg import ArmNavigationErrorCodes
from motion_planning_msgs.msg import AllowedContactSpecification
from motion_planning_msgs.msg import LinkPadding
from motion_planning_msgs.msg import OrderedCollisionOperations
from motion_planning_msgs.msg import CollisionOperation
from motion_planning_msgs.srv import GetMotionPlan
from motion_planning_msgs.srv import GetMotionPlanRequest
from motion_planning_msgs.srv import FilterJointTrajectory
from motion_planning_msgs.srv import FilterJointTrajectoryRequest

from move_arm_msgs.msg import MoveArmAction
from move_arm_msgs.msg import MoveArmGoal

from mapping_msgs.msg import AttachedCollisionObject
from mapping_msgs.msg import CollisionObjectOperation

from geometric_shapes_msgs.msg import Shape
from std_msgs.msg import Float64
from interpolated_ik_motion_planner.srv import SetInterpolatedIKMotionPlanParams

from object_detection import ObjectDetector


class ObjectCategorizer():
    def __init__(self):
        self.ARM_JOINTS = ('shoulder_pitch_joint',
                           'shoulder_pan_joint',
                           'upperarm_roll_joint',
                           'elbow_flex_joint',
                           'forearm_roll_joint',
                           'wrist_pitch_joint',
                           'wrist_roll_joint')
                           
        self.ARM_LINKS = ('L6_wrist_pitch_link',
                          'L5_forearm_roll_link',
                          'L4_elbow_flex_link',
                          'L3_upperarm_roll_link',
                          'L2_shoulder_pan_link',
                          'L1_shoulder_pitch_link')
                          
        self.GRIPPER_LINKS = ('L9_right_finger_link',
                              'L8_left_finger_link',
                              'L7_wrist_roll_link')
                              
        self.READY_POSITION = (-1.650, -1.465, 3.430, -0.970, -1.427,  0.337,  0.046)
        self.LIFT_POSITION  = (-1.049, -1.241, 0.669, -0.960, -0.409, -0.072, -0.143)
        self.PLACE_POSITION = ( 0.261, -0.704, 1.470,  0.337,  0.910, -1.667, -0.026)
        self.TUCK_POSITION  = (-1.971, -1.741, 0.021, -0.181, -1.841,  1.084,  0.148)
        
        self.arm_states = {
            'READY': self.READY_POSITION,
            'LIFT':  self.LIFT_POSITION,
            'PLACE': self.PLACE_POSITION,
            'TUCK':  self.TUCK_POSITION,
            'OTHER': None,
        }
        
        self.GRIPPER_LINK_FRAME = 'L7_wrist_roll_link'
        self.GRIPPER_GROUP_NAME = 'l_end_effector'
        self.ARM_GROUP_NAME = 'left_arm'
        
        self.object_detector = ObjectDetector()
        
        # connect to tabletop segmentation service
        rospy.loginfo('waiting for tabletop_segmentation service')
        rospy.wait_for_service('/tabletop_segmentation')
        self.tabletop_segmentation_srv = rospy.ServiceProxy('/tabletop_segmentation', TabletopSegmentation)
        rospy.loginfo('connected to tabletop_segmentation service')
        
        # connect to collision map processing service
        rospy.loginfo('waiting for tabletop_collision_map_processing service')
        rospy.wait_for_service('/tabletop_collision_map_processing/tabletop_collision_map_processing')
        self.collision_map_processing_srv = rospy.ServiceProxy('/tabletop_collision_map_processing/tabletop_collision_map_processing', TabletopCollisionMapProcessing)
        rospy.loginfo('connected to tabletop_collision_map_processing service')
        
        # connect to grasp planning service
        rospy.loginfo('waiting for GraspPlanning service')
        rospy.wait_for_service('/GraspPlanning')
        self.grasp_planning_srv = rospy.ServiceProxy('/GraspPlanning', GraspPlanning)
        rospy.loginfo('connected to GraspPlanning service')
        
        # connect to move_arm action server
        rospy.loginfo('waiting for move_left_arm action server')
        self.move_arm_client = SimpleActionClient('/move_left_arm', MoveArmAction)
        self.move_arm_client.wait_for_server()
        rospy.loginfo('connected to move_left_arm action server')
        
        # connect to kinematics solver service
        rospy.loginfo('waiting for wubble_left_arm_kinematics/get_ik_solver_info service')
        rospy.wait_for_service('/wubble_left_arm_kinematics/get_ik_solver_info')
        self.get_solver_info_srv = rospy.ServiceProxy('/wubble_left_arm_kinematics/get_ik_solver_info', GetKinematicSolverInfo)
        rospy.loginfo('connected to wubble_left_arm_kinematics/get_ik_solver_info service')
        
        rospy.loginfo('waiting for wubble_left_arm_kinematics/get_ik service')
        rospy.wait_for_service('/wubble_left_arm_kinematics/get_ik')
        self.get_ik_srv = rospy.ServiceProxy('/wubble_left_arm_kinematics/get_ik', GetPositionIK)
        rospy.loginfo('connected to wubble_left_arm_kinematics/get_ik service')
        
        rospy.loginfo('waiting for wubble_left_arm_kinematics/get_fk service')
        rospy.wait_for_service('/wubble_left_arm_kinematics/get_fk')
        self.get_fk_srv = rospy.ServiceProxy('/wubble_left_arm_kinematics/get_fk', GetPositionFK)
        rospy.loginfo('connected to wubble_left_arm_kinematics/get_fk service')
        
        # connect to gripper action server
        rospy.loginfo('waiting for wubble_gripper_grasp_action')
        self.posture_controller = SimpleActionClient('/wubble_gripper_grasp_action', GraspHandPostureExecutionAction)
        self.posture_controller.wait_for_server()
        rospy.loginfo('connected to wubble_gripper_grasp_action')
        
        rospy.loginfo('waiting for wubble_grasp_status service')
        rospy.wait_for_service('/wubble_grasp_status')
        self.get_grasp_status_srv = rospy.ServiceProxy('/wubble_grasp_status', GraspStatus)
        rospy.loginfo('connected to wubble_grasp_status service')
        
        # connect to gripper action server
        rospy.loginfo('waiting for wubble_gripper_action')
        self.gripper_controller = SimpleActionClient('/wubble_gripper_action', WubbleGripperAction)
        self.gripper_controller.wait_for_server()
        rospy.loginfo('connected to wubble_gripper_action')
        
        # connect to audio saving services
        rospy.loginfo('waiting for audio_dump/start_audio_recording service')
        rospy.wait_for_service('audio_dump/start_audio_recording')
        self.start_audio_recording_srv = rospy.ServiceProxy('/audio_dump/start_audio_recording', StartAudioRecording)
        rospy.loginfo('connected to audio_dump/start_audio_recording service')
        
        rospy.loginfo('waiting for audio_dump/stop_audio_recording service')
        rospy.wait_for_service('audio_dump/stop_audio_recording')
        self.stop_audio_recording_srv = rospy.ServiceProxy('/audio_dump/stop_audio_recording', StopAudioRecording)
        rospy.loginfo('connected to audio_dump/stop_audio_recording service')
        
        rospy.loginfo('waiting for wrist_roll_controller service')
        rospy.wait_for_service('/wrist_roll_controller/set_speed')
        self.wrist_roll_velocity_srv = rospy.ServiceProxy('/wrist_roll_controller/set_speed', SetSpeed)
        rospy.loginfo('connected to wrist_roll_controller service')
        
        rospy.loginfo('waiting for wrist_pitch_controller service')
        rospy.wait_for_service('/wrist_pitch_controller/set_speed')
        self.wrist_pitch_velocity_srv = rospy.ServiceProxy('/wrist_pitch_controller/set_speed', SetSpeed)
        rospy.loginfo('connected to wrist_pitch_controller service')
        
        # connect to interpolated IK services
        rospy.loginfo('waiting for l_interpolated_ik_motion_plan_set_params service')
        rospy.wait_for_service('/l_interpolated_ik_motion_plan_set_params')
        self.interpolated_ik_params_srv = rospy.ServiceProxy('/l_interpolated_ik_motion_plan_set_params', SetInterpolatedIKMotionPlanParams)
        rospy.loginfo('connected to l_interpolated_ik_motion_plan_set_params service')
        
        rospy.loginfo('waiting for l_interpolated_ik_motion_plan service')
        rospy.wait_for_service('/l_interpolated_ik_motion_plan')
        self.interpolated_ik_srv = rospy.ServiceProxy('/l_interpolated_ik_motion_plan', GetMotionPlan)
        rospy.loginfo('connected to l_interpolated_ik_motion_plan service')
        
        rospy.loginfo('waiting for l_arm_controller/joint_trajectory_action')
        self.trajectory_controller = SimpleActionClient('/l_arm_controller/joint_trajectory_action', JointTrajectoryAction)
        self.trajectory_controller.wait_for_server()
        rospy.loginfo('connected to l_arm_controller/joint_trajectory_action')
        
        rospy.loginfo('waiting for classify service')
        rospy.wait_for_service('/classify')
        self.classification_srv = rospy.ServiceProxy('/classify', classify)
        rospy.loginfo('connected to classify service')
        
        rospy.loginfo('waiting for trajectory_filter_unnormalizer/filter_trajectory service')
        rospy.wait_for_service('/trajectory_filter_unnormalizer/filter_trajectory')
        self.trajectory_filter_srv = rospy.ServiceProxy('/trajectory_filter_unnormalizer/filter_trajectory', FilterJointTrajectory)
        rospy.loginfo('connected to trajectory_filter_unnormalizer/filter_trajectory service')
        
        # will publish to wrist roll joint controller for roll action
        self.wrist_roll_command_pub = rospy.Publisher('wrist_roll_controller/command', Float64)
        self.wrist_pitch_command_pub = rospy.Publisher('wrist_pitch_controller/command', Float64)
        
        # will publish when objects are attached or detached to/from the gripper
        self.attached_object_pub = rospy.Publisher('/attached_collision_object', AttachedCollisionObject)
        
        # advertise InfoMax service
        rospy.Service('get_category_distribution', InfoMax, self.process_infomax_request)
        
        rospy.loginfo('all services contacted, object_categorization is ready to go')


    def lists_within_tolerance(self, list1, list2, tolerances):
        l1 = np.asarray(list1)
        l2 = np.asarray(list2)
        
        # are all absolute differences smaller than provided tolerances?
        return (np.abs(l1 - l2) < tolerances).all()


    def find_current_arm_state(self, tolerances=[0.04]*7):
        current_joint_positions = rospy.wait_for_message('l_arm_controller/state', JointTrajectoryControllerState, 2.0).actual.positions
        
        for state_name in self.arm_states:
            desired_joint_positions = self.arm_states[state_name]
            if self.lists_within_tolerance(desired_joint_positions, current_joint_positions, tolerances): return state_name


    def is_arm_in_state(self, state_name, tolerances=[0.04]*7):
        current_joint_positions = rospy.wait_for_message('l_arm_controller/state', JointTrajectoryControllerState, 2.0).actual.positions
        desired_joint_positions = self.arm_states[state_name]
        return self.lists_within_tolerance(desired_joint_positions, current_joint_positions, tolerances)


    def move_arm_joint_goal(self, joint_names, joint_positions, allowed_contacts=[], link_padding=[], collision_operations=OrderedCollisionOperations()):
        goal = MoveArmGoal()
        goal.planner_service_name = 'ompl_planning/plan_kinematic_path'
        goal.motion_plan_request.planner_id = ''
        goal.motion_plan_request.group_name = self.ARM_GROUP_NAME
        goal.motion_plan_request.num_planning_attempts = 3
        goal.motion_plan_request.allowed_planning_time = rospy.Duration(5.0)
        goal.motion_plan_request.goal_constraints.joint_constraints = [JointConstraint(j, p, 0.1, 0.1, 0.0) for (j,p) in zip(joint_names,joint_positions)]
        
        goal.motion_plan_request.allowed_contacts = allowed_contacts
        goal.motion_plan_request.link_padding = link_padding
        goal.motion_plan_request.ordered_collision_operations = collision_operations
        
        self.move_arm_client.send_goal(goal)
        finished_within_time = self.move_arm_client.wait_for_result(rospy.Duration(200.0))
        
        if not finished_within_time:
            self.move_arm_client.cancel_goal()
            rospy.loginfo('timed out trying to achieve joint goal')
            return False
        else:
            state = self.move_arm_client.get_state()
            
            if state == GoalStatus.SUCCEEDED:
                return True
            else:
                rospy.loginfo('failed to achieve joint goal (returned status code %s)' % str(state))
                return False


    def tuck_arm(self, collision_operations=OrderedCollisionOperations()):
        """
        Moves the arm to the side out of the view of all sensors and fully
        opens a gripper. In this position the arm is ready to perform a grasp
        action.
        """
        if self.move_arm_joint_goal(self.ARM_JOINTS, self.READY_POSITION, collision_operations=collision_operations):
            return True
        else:
            rospy.logerr('failed to tuck arm, aborting')
            return False


    def open_gripper(self):
        pg = GraspHandPostureExecutionGoal()
        pg.goal = GraspHandPostureExecutionGoal.RELEASE
        
        self.posture_controller.send_goal(pg)
        self.posture_controller.wait_for_result()


    def close_gripper(self):
        pg = GraspHandPostureExecutionGoal()
        pg.goal = GraspHandPostureExecutionGoal.GRASP
        
        self.posture_controller.send_goal(pg)
        self.posture_controller.wait_for_result()
        
        rospy.sleep(1)
        grasp_status = self.get_grasp_status_srv()
        return grasp_status.is_hand_occupied


    def gentle_close_gripper(self):
        self.close_gripper()
        
        goal = WubbleGripperGoal()
        goal.command = WubbleGripperGoal.CLOSE_GRIPPER
        goal.torque_limit = 0.0
        goal.dynamic_torque_control = False
        
        self.gripper_controller.send_goal(goal)
        self.gripper_controller.wait_for_result()


    def segment_objects_old(self):
        """
        Performs tabletop segmentation. If successful, returns a TabletopDetectionResult
        objct ready to be passed for processing to tabletop collision map processing node.
        """
        segmentation_result = self.tabletop_segmentation_srv()
        
        if segmentation_result.result != TabletopSegmentationResponse.SUCCESS or not segmentation_result.clusters:
            rospy.logerr('TabletopSegmentation did not find any clusters')
            return None
            
        rospy.loginfo('TabletopSegmentation found %d clusters' % len(segmentation_result.clusters))
        
        tdr = TabletopDetectionResult()
        tdr.table = segmentation_result.table
        tdr.clusters = segmentation_result.clusters
        tdr.result = segmentation_result.result
        
        return tdr


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
        req.reset_static_map = True
        req.take_static_collision_map = False
        
        self.collision_map_processing_srv(req)
        rospy.loginfo('collision map reset')


    def update_collision_map(self, tabletop_detection_result):
        rospy.loginfo('collision_map update in progress')
        
        req = TabletopCollisionMapProcessingRequest()
        req.detection_result = tabletop_detection_result
        req.reset_collision_models = True
        req.reset_attached_models = True
        req.reset_static_map = True
        req.take_static_collision_map = True
        req.desired_frame = 'base_link'
        
        res = self.collision_map_processing_srv(req)
        
        rospy.loginfo('collision_map update is done')
        rospy.loginfo('there are %d graspable objects %s, collision support surface name is "%s"' %
                      (len(res.graspable_objects), res.collision_object_names, res.collision_support_surface_name))
                      
        return res


    def find_ik_for_grasping_pose(self, pose_stamped):
        solver_info = self.get_solver_info_srv()
        arm_joints = solver_info.kinematic_solver_info.joint_names
        
        req = GetPositionIKRequest()
        req.timeout = rospy.Duration(5.0)
        req.ik_request.ik_link_name = self.GRIPPER_LINK_FRAME;
        req.ik_request.pose_stamped = pose_stamped
        
        try:
            current_state = rospy.wait_for_message('/joint_states', JointState, 2.0)
            
            req.ik_request.ik_seed_state.joint_state.name = arm_joints
            req.ik_request.ik_seed_state.joint_state.position = [current_state.position[current_state.name.index(j)] for j in arm_joints]
            
            ik_result = self.get_ik_srv(req)
            
            if ik_result.error_code.val == ArmNavigationErrorCodes.SUCCESS:
                return ik_result.solution
            else:
                rospy.logerr('failed to find an IK for requested grasping pose, aborting')
                return None
        except Exception as e:
            rospy.logerr('error occured: %s' % str(e))

    def find_grasp_pose(self, target, collision_object_name='', collision_support_surface_name=''):
        """
        target = GraspableObject
        collision_object_name = name of target in collision map
        collision_support_surface_name = name of surface target is touching
        """
        
        req = GraspPlanningRequest()
        req.arm_name = self.ARM_GROUP_NAME
        req.target = target
        req.collision_object_name = collision_object_name
        req.collision_support_surface_name = collision_support_surface_name
        
        rospy.loginfo('trying to find a good grasp for graspable object %s' % collision_object_name)
        grasping_result = self.grasp_planning_srv(req)
        
        if grasping_result.error_code.value != GraspPlanningErrorCode.SUCCESS:
            rospy.logerr('unable to find a feasable grasp, aborting')
            return None
            
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = grasping_result.grasps[0].grasp_posture.header.frame_id
        pose_stamped.pose = grasping_result.grasps[0].grasp_pose
        
        rospy.loginfo('found good grasp, looking for corresponding IK')
        
        return self.find_ik_for_grasping_pose(pose_stamped)


    def grasp(self, tabletop_collision_map_processing_result):
        closest_index = self.info[0][0]
        target = tabletop_collision_map_processing_result.graspable_objects[closest_index]
        collision_object_name = tabletop_collision_map_processing_result.collision_object_names[closest_index]
        collision_support_surface_name = tabletop_collision_map_processing_result.collision_support_surface_name
        
        ik_solution = self.find_grasp_pose(target, collision_object_name, collision_support_surface_name)
        
        if ik_solution:
            # disable collisions between gripper and target object
            ordered_collision_operations = OrderedCollisionOperations()
            collision_operation = CollisionOperation()
            collision_operation.object1 = collision_object_name
            collision_operation.object2 = self.GRIPPER_GROUP_NAME
            collision_operation.operation = CollisionOperation.DISABLE
            ordered_collision_operations.collision_operations = [collision_operation]
            
            # set gripper padding to 0
            gripper_paddings = [LinkPadding(l,0.0) for l in self.GRIPPER_LINKS]
            gripper_paddings.extend([LinkPadding(l,0.0) for l in self.ARM_LINKS])
            
            self.open_gripper()
            
            # move into pre-grasp pose
            if not self.move_arm_joint_goal(ik_solution.joint_state.name,
                                            ik_solution.joint_state.position,
                                            link_padding=gripper_paddings,
                                            collision_operations=ordered_collision_operations):
                rospy.logerr('InfomaxAction.GRASP: attempted grasp failed')
                return None
                
            # record grasping sound with 0.5 second padding before and after
            self.start_audio_recording_srv(InfomaxAction.GRASP, self.category_id)
            rospy.sleep(0.5)
            grasp_successful = self.close_gripper()
            rospy.sleep(0.5)
            
            # if grasp was successful, attach it to the gripper
            if grasp_successful:
                sound_msg = self.stop_audio_recording_srv(True)
                
                obj = AttachedCollisionObject()
                obj.object.header.stamp = rospy.Time.now()
                obj.object.header.frame_id = self.GRIPPER_LINK_FRAME
                obj.object.operation.operation = CollisionObjectOperation.ATTACH_AND_REMOVE_AS_OBJECT
                obj.object.id = collision_object_name
                obj.link_name = self.GRIPPER_LINK_FRAME
                obj.touch_links = self.GRIPPER_LINKS
                
                self.attached_object_pub.publish(obj)
                return sound_msg.recorded_sound
                
        self.stop_audio_recording_srv(False)
        rospy.logerr('InfomaxAction.GRASP: attempted grasp failed')
        return None


    def create_pose_stamped(self, pose, frame_id='base_link'):
        """
        Creates a PoseStamped message from a list of 7 numbers (first three are
        position and next four are orientation:
        pose = [px,py,pz, ox,oy,oz,ow]
        """
        m = PoseStamped()
        m.header.frame_id = frame_id
        m.header.stamp = rospy.Time()
        m.pose.position = Point(*pose[0:3])
        m.pose.orientation = Quaternion(*pose[3:7])
        return m


    #pretty-print list to string
    def pplist(self, list_to_print):
        return ' '.join(['%5.3f'%x for x in list_to_print])


    def get_interpolated_ik_motion_plan(self, start_pose, goal_pose, start_angles, joint_names, pos_spacing=0.01,
                                        rot_spacing=0.1, consistent_angle=math.pi/9, collision_aware=True,
                                        collision_check_resolution=1, steps_before_abort=-1, num_steps=0,
                                        ordered_collision_operations=None, frame='base_footprint', start_from_end=0,
                                        max_joint_vels=[1.5]*7, max_joint_accs=[8.0]*7):
                                        
        res = self.interpolated_ik_params_srv(num_steps,
                                              consistent_angle,
                                              collision_check_resolution,
                                              steps_before_abort,
                                              pos_spacing,
                                              rot_spacing,
                                              collision_aware,
                                              start_from_end,
                                              max_joint_vels,
                                              max_joint_accs)
                                              
        req = GetMotionPlanRequest()
        req.motion_plan_request.start_state.joint_state.name = joint_names
        req.motion_plan_request.start_state.joint_state.position = start_angles
        req.motion_plan_request.start_state.multi_dof_joint_state.poses = [start_pose.pose]
        req.motion_plan_request.start_state.multi_dof_joint_state.child_frame_ids = [self.GRIPPER_LINK_FRAME]
        req.motion_plan_request.start_state.multi_dof_joint_state.frame_ids = [start_pose.header.frame_id]
        
        pos_constraint = PositionConstraint()
        pos_constraint.position = goal_pose.pose.position
        pos_constraint.header.frame_id = goal_pose.header.frame_id
        req.motion_plan_request.goal_constraints.position_constraints = [pos_constraint,]
        
        orient_constraint = OrientationConstraint()
        orient_constraint.orientation = goal_pose.pose.orientation
        orient_constraint.header.frame_id = goal_pose.header.frame_id
        req.motion_plan_request.goal_constraints.orientation_constraints = [orient_constraint,]
        
        req.motion_plan_request.link_padding = [LinkPadding(l,0.0) for l in self.GRIPPER_LINKS]
        req.motion_plan_request.link_padding.extend([LinkPadding(l,0.0) for l in self.ARM_LINKS])
        
        if ordered_collision_operations is not None:
            req.motion_plan_request.ordered_collision_operations = ordered_collision_operations
            
        res = self.interpolated_ik_srv(req)
        return res


    def check_cartesian_path_lists(self, approachpos, approachquat, grasppos, graspquat, start_angles, pos_spacing=0.03,
                                   rot_spacing=0.1, consistent_angle=math.pi/7.0, collision_aware=True,
                                   collision_check_resolution=1, steps_before_abort=1, num_steps=0,
                                   ordered_collision_operations=None, frame='base_link'):
                                   
        start_pose = self.create_pose_stamped(approachpos+approachquat, frame)
        goal_pose = self.create_pose_stamped(grasppos+graspquat, frame)
        
        return self.get_interpolated_ik_motion_plan(start_pose, goal_pose, start_angles, self.ARM_JOINTS, pos_spacing, rot_spacing,
                                                    consistent_angle, collision_aware, collision_check_resolution,
                                                    steps_before_abort, num_steps, ordered_collision_operations, frame)


    # needs grasp action to be performed first
    def lift(self, tabletop_collision_map_processing_result):
        collision_support_surface_name = tabletop_collision_map_processing_result.collision_support_surface_name
        
        # disable collisions between grasped object and table
        collision_operation1 = CollisionOperation()
        collision_operation1.object1 = CollisionOperation.COLLISION_SET_ATTACHED_OBJECTS
        collision_operation1.object2 = collision_support_surface_name
        collision_operation1.operation = CollisionOperation.DISABLE
        
        # disable collisions between gripper and table
        collision_operation2 = CollisionOperation()
        collision_operation2.object1 = self.GRIPPER_GROUP_NAME
        collision_operation2.object2 = collision_support_surface_name
        collision_operation2.operation = CollisionOperation.DISABLE
        
        ordered_collision_operations = OrderedCollisionOperations()
        ordered_collision_operations.collision_operations = [collision_operation1, collision_operation2]
        
        gripper_paddings = [LinkPadding(l,0.0) for l in self.GRIPPER_LINKS]
        
        # this is a hack to make arm lift an object faster
        obj = AttachedCollisionObject()
        obj.object.header.stamp = rospy.Time.now()
        obj.object.header.frame_id = self.GRIPPER_LINK_FRAME
        obj.object.operation.operation = CollisionObjectOperation.REMOVE
        obj.object.id = collision_support_surface_name
        obj.link_name = self.GRIPPER_LINK_FRAME
        obj.touch_links = self.GRIPPER_LINKS
        self.attached_object_pub.publish(obj)
        
        current_state = rospy.wait_for_message('l_arm_controller/state', JointTrajectoryControllerState)
        start_angles = list(current_state.actual.positions)
        start_angles[0] = start_angles[0] - 0.3  # move shoulder up a bit
        
        if not self.move_arm_joint_goal(self.ARM_JOINTS,
                                        start_angles,
                                        link_padding=gripper_paddings,
                                        collision_operations=ordered_collision_operations):
                                            return None
                                            
        self.start_audio_recording_srv(InfomaxAction.LIFT, self.category_id)
        
        if self.move_arm_joint_goal(self.ARM_JOINTS,
                                    self.LIFT_POSITION,
                                    link_padding=gripper_paddings,
                                    collision_operations=ordered_collision_operations):
            # check if are still holding an object after lift is done
            grasp_status = self.get_grasp_status_srv()
            
            # if the object is still in hand after lift is done we are good
            if grasp_status.is_hand_occupied:
                sound_msg = self.stop_audio_recording_srv(True)
                return sound_msg.recorded_sound
                
        self.stop_audio_recording_srv(False)
        rospy.logerr('InfomaxAction.LIFT: attempted lift failed')
        return None


    # needs grasp and lift to be performed first
    def drop(self, tabletop_collision_map_processing_result):
        closest_index = self.info[0][0]
        collision_object_name = tabletop_collision_map_processing_result.collision_object_names[closest_index]
        
        # check that we have something in hand before dropping it
        grasp_status = self.get_grasp_status_srv()
        
        # if the object is still in hand after lift is done we are good
        if not grasp_status.is_hand_occupied:
            rospy.logerr('InfomaxAction.DROP: gripper empty, nothing to drop')
            return None
            
        # record sound padded by 0.5 seconds from start and 1.5 seconds from the end
        self.start_audio_recording_srv(InfomaxAction.DROP, self.category_id)
        rospy.sleep(0.5)
        self.open_gripper()
        rospy.sleep(1.5)
        
        # check if gripper actually opened first
        sound_msg = None
        grasp_status = self.get_grasp_status_srv()
        
        # if there something in the gripper - drop failed
        if grasp_status.is_hand_occupied:
            self.stop_audio_recording_srv(False)
        else:
            sound_msg = self.stop_audio_recording_srv(True)
            
        # delete the object that we just dropped, we don't really know where it will land
        obj = AttachedCollisionObject()
        obj.object.header.stamp = rospy.Time.now()
        obj.object.header.frame_id = self.GRIPPER_LINK_FRAME
        obj.object.operation.operation = CollisionObjectOperation.REMOVE
        obj.object.id = collision_object_name
        obj.link_name = self.GRIPPER_LINK_FRAME
        obj.touch_links = self.GRIPPER_LINKS
        
        self.attached_object_pub.publish(obj)
        
        if sound_msg: return sound_msg.recorded_sound
        else: return None


    def place(self, tabletop_collision_map_processing_result):
        closest_index = self.info[0][0]
        collision_object_name = tabletop_collision_map_processing_result.collision_object_names[closest_index]
        collision_support_surface_name = tabletop_collision_map_processing_result.collision_support_surface_name
        
        # check that we have something in hand before placing it
        grasp_status = self.get_grasp_status_srv()
        
        # if the object is still in hand after lift is done we are good
        if not grasp_status.is_hand_occupied:
            rospy.logerr('InfomaxAction.PLACE: gripper empty, nothing to place')
            return None
            
        gripper_paddings = [LinkPadding(l,0.0) for l in self.GRIPPER_LINKS]
        
        # disable collisions between attached object and table
        collision_operation1 = CollisionOperation()
        collision_operation1.object1 = CollisionOperation.COLLISION_SET_ATTACHED_OBJECTS
        collision_operation1.object2 = collision_support_surface_name
        collision_operation1.operation = CollisionOperation.DISABLE
        
        # disable collisions between gripper and table
        collision_operation2 = CollisionOperation()
        collision_operation2.object1 = collision_support_surface_name
        collision_operation2.object2 = self.GRIPPER_GROUP_NAME
        collision_operation2.operation = CollisionOperation.DISABLE
        collision_operation2.penetration_distance = 0.01
        
        # disable collisions between arm and table
        collision_operation3 = CollisionOperation()
        collision_operation3.object1 = collision_support_surface_name
        collision_operation3.object2 = self.ARM_GROUP_NAME
        collision_operation3.operation = CollisionOperation.DISABLE
        collision_operation3.penetration_distance = 0.01
        
        ordered_collision_operations = OrderedCollisionOperations()
        ordered_collision_operations.collision_operations = [collision_operation1, collision_operation2, collision_operation3]
        
        self.start_audio_recording_srv(InfomaxAction.PLACE, self.category_id)
        
        if self.move_arm_joint_goal(self.ARM_JOINTS,
                                    self.PLACE_POSITION,
                                    link_padding=gripper_paddings,
                                    collision_operations=ordered_collision_operations):
            sound_msg = None
            grasp_status = self.get_grasp_status_srv()
            
            self.open_gripper()
            rospy.sleep(0.5)
            
            # if after lowering arm gripper is still holding an object life's good
            if grasp_status.is_hand_occupied:
                sound_msg = self.stop_audio_recording_srv(True)
            else:
                self.stop_audio_recording_srv(False)
                
            obj = AttachedCollisionObject()
            obj.object.header.stamp = rospy.Time.now()
            obj.object.header.frame_id = self.GRIPPER_LINK_FRAME
            obj.object.operation.operation = CollisionObjectOperation.DETACH_AND_ADD_AS_OBJECT
            obj.object.id = collision_object_name
            obj.link_name = self.GRIPPER_LINK_FRAME
            obj.touch_links = self.GRIPPER_LINKS
            self.attached_object_pub.publish(obj)
            
            if sound_msg: return sound_msg.recorded_sound
            else: return None
            
        self.stop_audio_recording_srv(False)
        rospy.logerr('InfomaxAction.PLACE: attempted place failed')
        return None


    def push(self, tabletop_collision_map_processing_result):
        closest_index = self.info[0][0]
        collision_object_name = tabletop_collision_map_processing_result.collision_object_names[closest_index]
        collision_support_surface_name = tabletop_collision_map_processing_result.collision_support_surface_name
        
        current_state = rospy.wait_for_message('l_arm_controller/state', JointTrajectoryControllerState)
        start_angles = current_state.actual.positions
        
        full_state = rospy.wait_for_message('/joint_states', JointState)
        
        req = GetPositionFKRequest()
        req.header.frame_id = 'base_link'
        req.fk_link_names = [self.GRIPPER_LINK_FRAME]
        req.robot_state.joint_state = full_state
        pose = self.get_fk_srv(req).pose_stamped[0].pose
        
        frame_id = 'base_link'
        
        approachpos =  [pose.position.x, pose.position.y, pose.position.z]
        approachquat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        
        push_distance = 0.40
        grasppos =  [pose.position.x, pose.position.y-push_distance, pose.position.z]
        graspquat = [0.00000, 0.00000, 0.70711, -0.70711]
        
#        req = GetPositionFKRequest()
#        req.header.frame_id = self.GRIPPER_LINK_FRAME
#        req.fk_link_names = [self.GRIPPER_LINK_FRAME]
#        req.robot_state.joint_state = full_state
#        pose = self.get_fk_srv(req).pose_stamped[0].pose
#        
#        frame_id = self.GRIPPER_LINK_FRAME
#        
#        approachpos =  [0, 0, 0]
#        approachquat = [0, 0, 0, 1]
#        
#        push_distance = 0.30
#        grasppos =  [push_distance, 0, 0]
#        graspquat = approachquat[:]
        
        # attach object to gripper, they will move together
        obj = AttachedCollisionObject()
        obj.object.header.stamp = rospy.Time.now()
        obj.object.header.frame_id = self.GRIPPER_LINK_FRAME
        obj.object.operation.operation = CollisionObjectOperation.ATTACH_AND_REMOVE_AS_OBJECT
        obj.object.id = collision_object_name
        obj.link_name = self.GRIPPER_LINK_FRAME
        obj.touch_links = self.GRIPPER_LINKS
        
        self.attached_object_pub.publish(obj)
        
        # disable collisions between attached object and table
        collision_operation1 = CollisionOperation()
        collision_operation1.object1 = CollisionOperation.COLLISION_SET_ATTACHED_OBJECTS
        collision_operation1.object2 = collision_support_surface_name
        collision_operation1.operation = CollisionOperation.DISABLE
        
        collision_operation2 = CollisionOperation()
        collision_operation2.object1 = collision_support_surface_name
        collision_operation2.object2 = self.GRIPPER_GROUP_NAME
        collision_operation2.operation = CollisionOperation.DISABLE
        collision_operation2.penetration_distance = 0.02
        
        ordered_collision_operations = OrderedCollisionOperations()
        ordered_collision_operations.collision_operations = [collision_operation1, collision_operation2]
        
        res = self.check_cartesian_path_lists(approachpos,
                                              approachquat,
                                              grasppos,
                                              graspquat,
                                              start_angles,
                                              frame=frame_id,
                                              ordered_collision_operations=ordered_collision_operations)
                                              
        error_code_dict = { ArmNavigationErrorCodes.SUCCESS: 0,
                            ArmNavigationErrorCodes.COLLISION_CONSTRAINTS_VIOLATED: 1,
                            ArmNavigationErrorCodes.PATH_CONSTRAINTS_VIOLATED: 2,
                            ArmNavigationErrorCodes.JOINT_LIMITS_VIOLATED: 3,
                            ArmNavigationErrorCodes.PLANNING_FAILED: 4 }
                            
        trajectory_len = len(res.trajectory.joint_trajectory.points)
        trajectory = [res.trajectory.joint_trajectory.points[i].positions for i in range(trajectory_len)]
        vels = [res.trajectory.joint_trajectory.points[i].velocities for i in range(trajectory_len)]
        times = [res.trajectory.joint_trajectory.points[i].time_from_start for i in range(trajectory_len)]
        error_codes = [error_code_dict[error_code.val] for error_code in res.trajectory_error_codes]
        
        # if even one code is not 0, abort
        if sum(error_codes) != 0:
            for ind in range(len(trajectory)):
                rospy.loginfo("error code "+ str(error_codes[ind]) + " pos : " + self.pplist(trajectory[ind]))
                
            rospy.loginfo("")
            
            for ind in range(len(trajectory)):
                rospy.loginfo("time: " + "%5.3f  "%times[ind].to_sec() + "vels: " + self.pplist(vels[ind]))
                
            rospy.logerr('InfomaxAction.PUSH: attempted push failed')
            return None
            
        req = FilterJointTrajectoryRequest()
        req.trajectory = res.trajectory.joint_trajectory
        req.trajectory.points = req.trajectory.points[1:] # skip zero velocity point
        req.allowed_time = rospy.Duration(2.0)
        
        filt_res = self.trajectory_filter_srv(req)
        
        goal = JointTrajectoryGoal()
        goal.trajectory = filt_res.trajectory
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
        
        self.start_audio_recording_srv(InfomaxAction.PUSH, self.category_id)
        rospy.sleep(0.5)
        
        self.trajectory_controller.send_goal(goal)
        self.trajectory_controller.wait_for_result()
        
        state = self.trajectory_controller.get_state()
        
        if state == GoalStatus.SUCCEEDED:
            rospy.sleep(0.5)
            sound_msg = self.stop_audio_recording_srv(True)
            return sound_msg.recorded_sound
            
        rospy.logerr('InfomaxAction.PUSH: attempted push failed')
        self.stop_audio_recording_srv(False)
        return None


    def shake_roll(self, tabletop_collision_map_processing_result):
        wrist_roll_state = '/wrist_roll_controller/state'
        desired_velocity = 11.0
        distance = 2.5
        threshold = 0.2
        timeout = 2.0
        
        try:
            msg = rospy.wait_for_message(wrist_roll_state, DynamixelJointState, timeout)
            current_pos = msg.current_pos
            start_pos = current_pos
            
            # set wrist to initial position
            self.wrist_roll_velocity_srv(3.0)
            self.wrist_roll_command_pub.publish(distance)
            end_time = rospy.Time.now() + rospy.Duration(timeout)
            
            while current_pos < distance-threshold and rospy.Time.now() < end_time:
                msg = rospy.wait_for_message(wrist_roll_state, DynamixelJointState, timeout)
                current_pos = msg.current_pos
                rospy.sleep(10e-3)
                
            self.wrist_roll_velocity_srv(desired_velocity)
            
            # start recording sound and shaking
            self.start_audio_recording_srv(InfomaxAction.SHAKE_ROLL, self.category_id)
            rospy.sleep(0.5)
            
            for i in range(2):
                self.wrist_roll_command_pub.publish(-distance)
                end_time = rospy.Time.now() + rospy.Duration(timeout)
                
                while current_pos > -distance+threshold and rospy.Time.now() < end_time:
                    msg = rospy.wait_for_message(wrist_roll_state, DynamixelJointState, timeout)
                    current_pos = msg.current_pos
                    rospy.sleep(10e-3)
                    
                self.wrist_roll_command_pub.publish(distance)
                end_time = rospy.Time.now() + rospy.Duration(timeout)
                
                while current_pos < distance-threshold and rospy.Time.now() < end_time:
                    msg = rospy.wait_for_message(wrist_roll_state, DynamixelJointState, timeout)
                    current_pos = msg.current_pos
                    rospy.sleep(10e-3)
                    
            rospy.sleep(0.5)
            
            # check if are still holding an object after shaking
            sound_msg = None
            grasp_status = self.get_grasp_status_srv()
            
            if grasp_status.is_hand_occupied:
                sound_msg = self.stop_audio_recording_srv(True)
            else:
                self.stop_audio_recording_srv(False)
                
            # reset wrist to starting position
            self.wrist_roll_velocity_srv(3.0)
            self.wrist_roll_command_pub.publish(start_pos)
            end_time = rospy.Time.now() + rospy.Duration(timeout)
            
            while current_pos < distance-threshold and rospy.Time.now() < end_time:
                msg = rospy.wait_for_message(wrist_roll_state, DynamixelJointState, timeout)
                current_pos = msg.current_pos
                rospy.sleep(10e-3)
                
            rospy.sleep(0.5)
            
            if sound_msg:
                return sound_msg.recorded_sound
            else:
                return None
        except ROSException as e:
            rospy.logerr('InfomaxAction.SHAKE_ROLL: attempted roll failed - %s' % str(e))
            self.stop_audio_recording_srv(False)
            return None


    def shake_pitch(self, tabletop_collision_map_processing_result):
        wrist_pitch_state = '/wrist_pitch_controller/state'
        desired_velocity = 6.0
        distance = 1.0
        threshold = 0.1
        timeout = 2.0
        
        try:
            msg = rospy.wait_for_message(wrist_pitch_state, DynamixelJointState, timeout)
            current_pos = msg.current_pos
            start_pos = current_pos
            
            # set wrist to initial position
            self.wrist_pitch_velocity_srv(3.0)
            self.wrist_pitch_command_pub.publish(distance)
            end_time = rospy.Time.now() + rospy.Duration(timeout)
            
            while current_pos < distance-threshold and rospy.Time.now() < end_time:
                msg = rospy.wait_for_message(wrist_pitch_state, DynamixelJointState, timeout)
                current_pos = msg.current_pos
                rospy.sleep(10e-3)
                
            self.wrist_pitch_velocity_srv(desired_velocity)
            
            # start recording sound and shaking
            self.start_audio_recording_srv(InfomaxAction.SHAKE_PITCH, self.category_id)
            rospy.sleep(0.5)
            
            for i in range(2):
                self.wrist_pitch_command_pub.publish(-distance)
                end_time = rospy.Time.now() + rospy.Duration(timeout)
                
                while current_pos > -distance+threshold and rospy.Time.now() < end_time:
                    msg = rospy.wait_for_message(wrist_pitch_state, DynamixelJointState, timeout)
                    current_pos = msg.current_pos
                    rospy.sleep(10e-3)
                    
                self.wrist_pitch_command_pub.publish(distance)
                end_time = rospy.Time.now() + rospy.Duration(timeout)
                
                while current_pos < distance-threshold and rospy.Time.now() < end_time:
                    msg = rospy.wait_for_message(wrist_pitch_state, DynamixelJointState, timeout)
                    current_pos = msg.current_pos
                    rospy.sleep(10e-3)
                    
            rospy.sleep(0.5)
            
            # check if are still holding an object after shaking
            sound_msg = None
            grasp_status = self.get_grasp_status_srv()
            
            if grasp_status.is_hand_occupied:
                sound_msg = self.stop_audio_recording_srv(True)
            else:
                self.stop_audio_recording_srv(False)
                
            # reset wrist to starting position
            self.wrist_pitch_velocity_srv(3.0)
            self.wrist_pitch_command_pub.publish(start_pos)
            end_time = rospy.Time.now() + rospy.Duration(timeout)
            
            while current_pos < distance-threshold and rospy.Time.now() < end_time:
                msg = rospy.wait_for_message(wrist_pitch_state, DynamixelJointState, timeout)
                current_pos = msg.current_pos
                rospy.sleep(10e-3)
                
            rospy.sleep(0.5)
            
            if sound_msg:
                return sound_msg.recorded_sound
            else:
                return None
        except ROSException as e:
            rospy.logerr('InfomaxAction.SHAKE_PITCH: attempted pitch failed - %s' % str(e))
            self.stop_audio_recording_srv(False)
            return None


    def reset_robot(self, tabletop_collision_map_processing_result=None):
        rospy.loginfo('resetting robot')
        ordered_collision_operations = OrderedCollisionOperations()
        
        if tabletop_collision_map_processing_result:
            closest_index = self.info[0][0]
            collision_object_name = tabletop_collision_map_processing_result.collision_object_names[closest_index]
            collision_support_surface_name = tabletop_collision_map_processing_result.collision_support_surface_name
            
            collision_operation = CollisionOperation()
            collision_operation.object1 = collision_support_surface_name
            collision_operation.object2 = self.ARM_GROUP_NAME
            collision_operation.operation = CollisionOperation.DISABLE
            collision_operation.penetration_distance = 0.02
            ordered_collision_operations.collision_operations = [collision_operation]
            
            collision_operation = CollisionOperation()
            collision_operation.object1 = collision_support_surface_name
            collision_operation.object2 = self.GRIPPER_GROUP_NAME
            collision_operation.operation = CollisionOperation.DISABLE
            collision_operation.penetration_distance = 0.02
            ordered_collision_operations.collision_operations.append(collision_operation)
            
            collision_operation = CollisionOperation()
            collision_operation.object1 = collision_object_name
            collision_operation.object2 = self.GRIPPER_GROUP_NAME
            collision_operation.operation = CollisionOperation.DISABLE
            collision_operation.penetration_distance = 0.02
            ordered_collision_operations.collision_operations.append(collision_operation)
        else:
            self.reset_collision_map()
            
        current_state = self.find_current_arm_state()
        rospy.loginfo('arm is currently in %s state' % current_state)
        
        if current_state not in ['READY', 'TUCK']:
            rospy.loginfo('arm is not in any of the following states %s, opening gripper' % str(['READY', 'TUCK']))
            self.open_gripper()
            
        if current_state != 'TUCK' and not self.tuck_arm(ordered_collision_operations): return False
        self.gentle_close_gripper()
        return True


    # receive an InfoMax service request containing object ID and desired action
    def process_infomax_request(self, req):
        self.object_names = req.objectNames
        self.action_names = req.actionNames
        self.num_categories = req.numCats
        self.category_id = req.catID
        
        prereqs = {InfomaxAction.GRASP:       [self.grasp],
                   InfomaxAction.LIFT:        [self.grasp, self.lift],
                   InfomaxAction.SHAKE_ROLL:  [self.grasp, self.lift, self.shake_roll],
                   InfomaxAction.DROP:        [self.grasp, self.lift, self.drop],
                   InfomaxAction.PLACE:       [self.grasp, self.lift, self.place],
                   InfomaxAction.PUSH:        [self.grasp, self.lift, self.place, self.push],
                   InfomaxAction.SHAKE_PITCH: [self.grasp, self.lift, self.shake_pitch]}
                   
        if not self.reset_robot(): return None
        
        # find a graspable object on the floor
        tcmpr = self.segment_objects()
        if tcmpr is None: return None
        
        # mark floor and object poitions in the collision map as known
        #tcmpr = self.update_collision_map(tdr)
        
        # initialize as uniform distribution
        beliefs = [1.0/self.num_categories] * self.num_categories
        actions = prereqs[req.actionID.val]
        
        for act in actions:
            sound = act(tcmpr)
            
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

