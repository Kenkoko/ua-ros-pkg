#!/usr/bin/env python

# Author: Antons Rebguns

import roslib; roslib.load_manifest('w2_object_manipulation_launch')
import rospy

from ua_audio_infomax.msg import Action as InfomaxAction
from ua_audio_infomax.srv import InfoMax
from ua_audio_infomax.srv import InfoMaxResponse
from ua_audio_capture.srv import StartAudioRecording
from ua_audio_capture.srv import StopAudioRecording
from ua_audio_capture.srv import classify
from ua_controller_msgs.msg import JointState as DynamixelJointState
from ax12_controller_core.srv import SetSpeed

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
from geometry_msgs.msg import Point32
from sensor_msgs.msg import JointState

from kinematics_msgs.srv import GetKinematicSolverInfo
from kinematics_msgs.srv import GetPositionIK
from kinematics_msgs.srv import GetPositionIKRequest

from motion_planning_msgs.msg import JointConstraint
from motion_planning_msgs.msg import ArmNavigationErrorCodes
from motion_planning_msgs.msg import AllowedContactSpecification
from motion_planning_msgs.msg import LinkPadding
from motion_planning_msgs.msg import OrderedCollisionOperations
from motion_planning_msgs.msg import CollisionOperation

from move_arm_msgs.msg import MoveArmAction
from move_arm_msgs.msg import MoveArmGoal

from mapping_msgs.msg import AttachedCollisionObject
from mapping_msgs.msg import CollisionObjectOperation

from geometric_shapes_msgs.msg import Shape
from std_msgs.msg import Float64


class ObjectCategorizer():
    def __init__(self):
        self.joint_names = ('shoulder_pitch_joint',
                            'shoulder_pan_joint',
                            'upperarm_roll_joint',
                            'elbow_flex_joint',
                            'forearm_roll_joint',
                            'wrist_pitch_joint',
                            'wrist_roll_joint')
                            
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
        
        # connect to gripper action server
        rospy.loginfo('waiting for wubble_gripper_grasp_action')
        self.posture_controller = SimpleActionClient('/wubble_gripper_grasp_action', GraspHandPostureExecutionAction)
        self.posture_controller.wait_for_server()
        rospy.loginfo('connected to wubble_gripper_grasp_action')
        
        rospy.loginfo('waiting for wubble_grasp_status service')
        rospy.wait_for_service('/wubble_grasp_status')
        self.get_grasp_status_srv = rospy.ServiceProxy('/wubble_grasp_status', GraspStatus)
        rospy.loginfo('connected to wubble_grasp_status service')
        
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
        
        rospy.loginfo('waiting for classify service')
        rospy.wait_for_service('/classify')
        self.classification_srv = rospy.ServiceProxy('/classify', classify)
        rospy.loginfo('connected to classify service')
        
        # will publish to wrist roll joint controller for roll action
        self.wrist_roll_command_pub = rospy.Publisher('wrist_roll_controller/command', Float64)
        
        # will publish when objects are attached or detached to/from the gripper
        self.attached_object_pub = rospy.Publisher('/attached_collision_object', AttachedCollisionObject)
        
        # advertise InfoMax service
        rospy.Service('get_category_distribution', InfoMax, self.process_infomax_request)
        
        rospy.loginfo('all service contacted, object_categorization is ready to go')

    def move_arm_joint_goal(self, joint_names, joint_positions, allowed_contacts=[], link_padding=[], collision_operations=OrderedCollisionOperations()):
        goal = MoveArmGoal()
        goal.planner_service_name = 'ompl_planning/plan_kinematic_path'
        goal.motion_plan_request.planner_id = ''
        goal.motion_plan_request.group_name = 'left_arm'
        goal.motion_plan_request.num_planning_attempts = 1
        goal.motion_plan_request.allowed_planning_time = rospy.Duration(5.0)
        goal.motion_plan_request.goal_constraints.joint_constraints = [JointConstraint(j, p, 0.1, 0.1, 0.0) for (j,p) in zip(joint_names,joint_positions)]
        
        goal.motion_plan_request.allowed_contacts = allowed_contacts
        goal.motion_plan_request.link_padding = link_padding
        goal.motion_plan_request.ordered_collision_operations = collision_operations
        
        self.move_arm_client.send_goal(goal)
        finished_within_time = self.move_arm_client.wait_for_result(rospy.Duration(200.0))
        
        if not finished_within_time:
            self.move_arm_client.cancel_goal()
            rospy.loginfo('timed out trying to achieve a joint goal')
            return False
        else:
            state = self.move_arm_client.get_state()
            
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo('action finished: %s' % str(state))
                return True
            else:
                rospy.loginfo('action failed: %s' % str(state))
                return False

    def tuck_arm(self):
        retries = 3
        joint_positions = (-1.9715, -1.7406, 0.0213, -0.1807, -1.8408, 1.0840, 0.1483)
        
        for trial in range(retries):
            rospy.loginfo('tucking left arm: trial #%d', trial+1)
            if self.move_arm_joint_goal(self.joint_names, joint_positions): return True
        else:
            rospy.loginfo('failed to tuck arm after %d trials, aborting', retries)
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
        
        grasp_status = self.get_grasp_status_srv()
        return grasp_status.is_hand_occupied

    def ready_arm(self):
        retries = 3
        joint_positions = (-1.760, -1.760, 0.659, -0.795, -2.521, 0.619, -0.148)
        
        for trial in range(retries):
            rospy.loginfo('readying left arm: trial #%d', trial+1)
            if self.move_arm_joint_goal(self.joint_names, joint_positions):
                self.open_gripper()
                return True
        else:
            rospy.loginfo('failed to ready arm after %d trials, aborting', retries)
            return False

    def segment_objects(self):
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
        req.ik_request.ik_link_name = 'L7_wrist_roll_link';
        req.ik_request.pose_stamped = pose_stamped
        
        rospy.loginfo('waiting for current joint states')
        current_state = rospy.wait_for_message('/joint_states', JointState)
        rospy.loginfo('recevied current joint states')
        
        req.ik_request.ik_seed_state.joint_state.name = arm_joints
        req.ik_request.ik_seed_state.joint_state.position = [current_state.position[current_state.name.index(j)] for j in arm_joints]
        
        rospy.logdebug('current joint positions are')
        rospy.logdebug('joint names: %s' % str(req.ik_request.ik_seed_state.joint_state.name))
        rospy.logdebug('joint state: %s' % str(req.ik_request.ik_seed_state.joint_state.position))
        
        ik_result = self.get_ik_srv(req)
        
        if ik_result.error_code.val == ArmNavigationErrorCodes.SUCCESS:
            rospy.loginfo('found IK solution for given grasping pose')
            rospy.logdebug('solution joints: %s' % str(ik_result.solution.joint_state.name))
            rospy.logdebug('solution angles: %s' % str(ik_result.solution.joint_state.position))
            return ik_result.solution
        else:
            rospy.logerr('Inverse kinematics failed')
            return None

    def find_grasp_pose(self, target, collision_object_name='', collision_support_surface_name=''):
        """
        target = GraspableObject
        collision_object_name = name of target in collision map
        collision_support_surface_name = name of surface target is touching
        """
        
        req = GraspPlanningRequest()
        req.arm_name = 'left_arm'
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
        target = tabletop_collision_map_processing_result.graspable_objects[0]
        collision_object_name = tabletop_collision_map_processing_result.collision_object_names[0]
        collision_support_surface_name = tabletop_collision_map_processing_result.collision_support_surface_name
        
        ik_solution = self.find_grasp_pose(target, collision_object_name, collision_support_surface_name)
        
        if ik_solution:
            ordered_collision_operations = OrderedCollisionOperations()
            collision_operation = CollisionOperation()
            collision_operation.object1 = CollisionOperation.COLLISION_SET_OBJECTS
            collision_operation.object2 = 'l_end_effector'
            collision_operation.operation = CollisionOperation.DISABLE
            collision_operation.penetration_distance = 0.02
            ordered_collision_operations.collision_operations = [collision_operation]
            
            gripper_paddings = [LinkPadding(l,0.0) for l in ('L9_right_finger_link', 'L8_left_finger_link', 'L7_wrist_roll_link')]
            
            retries = 3
            for trial in range(retries):
                if self.move_arm_joint_goal(ik_solution.joint_state.name, ik_solution.joint_state.position, link_padding=gripper_paddings, collision_operations=ordered_collision_operations):
                    break
            else:
                rospy.logerr('failed to move arm to grasping position after %d trials' % retries)
                return None
                
            rospy.sleep(0.5)
            
            # record grasping sound with 0.5 second padding before and after
            self.start_audio_recording_srv(InfomaxAction.GRASP, self.category_id)
            rospy.sleep(0.5)
            grasp_status = self.close_gripper()
            rospy.sleep(0.5)
            
            sound = self.stop_audio_recording_srv(grasp_status)
            resp = self.classification_srv(sound.recorded_sound)
            print 'GRASP RESULT'
            print resp
            
            obj = AttachedCollisionObject()
            obj.object.header.stamp = rospy.Time.now()
            obj.object.header.frame_id = 'L7_wrist_roll_link'
            obj.object.operation.operation = CollisionObjectOperation.ATTACH_AND_REMOVE_AS_OBJECT
            obj.object.id = collision_object_name
            obj.link_name = 'L7_wrist_roll_link'
            obj.touch_links = ['L7_wrist_roll_link', 'L8_left_finger_link', 'L9_right_finger_link']
            
            self.attached_object_pub.publish(obj)
            rospy.sleep(2)
            return resp.beliefs
        else:
            rospy.logerr('failed to find an IK for requested grasping pose, aborting')
            return None

    # needs grasp action to be performed first
    def lift(self, tabletop_collision_map_processing_result):
        collision_support_surface_name = tabletop_collision_map_processing_result.collision_support_surface_name
        
        ordered_collision_operations = OrderedCollisionOperations()
        collision_operation = CollisionOperation()
        collision_operation.object1 = CollisionOperation.COLLISION_SET_ATTACHED_OBJECTS
        collision_operation.object2 = collision_support_surface_name
        collision_operation.operation = CollisionOperation.DISABLE
        ordered_collision_operations.collision_operations = [collision_operation]
        
        gripper_paddings = [LinkPadding(l,0.0) for l in ('L9_right_finger_link', 'L8_left_finger_link', 'L7_wrist_roll_link')]
        joint_positions = (-1.049, -1.241, 0.669, -0.960, -0.409, -0.072, -0.143)
        
        retries = 3
        self.start_audio_recording_srv(InfomaxAction.LIFT, self.category_id)
        
        for trial in range(retries):
            if self.move_arm_joint_goal(self.joint_names, joint_positions, link_padding=gripper_paddings, collision_operations=ordered_collision_operations):
                sound = self.stop_audio_recording_srv(True)
                resp = self.classification_srv(sound.recorded_sound)
                
                print 'LIFT RESULT'
                print resp
                return resp.beliefs
        else:
            self.stop_audio_recording_srv(False)
            rospy.logerr('failed to lift arm after %d trials' % retries)
            return None

    # needs grasp and lift to be performed first
    def drop(self, tabletop_collision_map_processing_result):
        collision_object_name = tabletop_collision_map_processing_result.collision_object_names[0]
        
        self.start_audio_recording_srv(InfomaxAction.DROP, self.category_id)
        rospy.sleep(0.5)
        self.open_gripper()
        rospy.sleep(1.5)
        sound = self.stop_audio_recording_srv(True)
        resp = self.classification_srv(sound.recorded_sound)
        print 'DROP RESULT'
        print resp
        
        # delete the object that we just dropped since don't really know where it will land
        obj = AttachedCollisionObject()
        obj.object.header.stamp = rospy.Time.now()
        obj.object.header.frame_id = 'L7_wrist_roll_link'
        obj.object.operation.operation = CollisionObjectOperation.REMOVE
        obj.object.id = collision_object_name
        obj.link_name = 'L7_wrist_roll_link'
        obj.touch_links = ['L7_wrist_roll_link', 'L8_left_finger_link', 'L9_right_finger_link']
        
        self.attached_object_pub.publish(obj)
        rospy.sleep(2)
        
        return resp.beliefs

    def place(self, tabletop_collision_map_processing_result):
        collision_object_name = tabletop_collision_map_processing_result.collision_object_names[0]
        collision_support_surface_name = tabletop_collision_map_processing_result.collision_support_surface_name
        
        gripper_paddings = [LinkPadding(l,0.0) for l in ('L9_right_finger_link', 'L8_left_finger_link', 'L7_wrist_roll_link')]
        joint_positions = (0.261, -0.704, 1.470, 0.337, 0.910, -1.667, -0.026)
        
        # move arm to target pose and disable collisions between the object an the table
        ordered_collision_operations = OrderedCollisionOperations()
        collision_operation = CollisionOperation()
        collision_operation.object1 = CollisionOperation.COLLISION_SET_ATTACHED_OBJECTS
        collision_operation.object2 = collision_support_surface_name
        collision_operation.operation = CollisionOperation.DISABLE
        ordered_collision_operations.collision_operations = [collision_operation]
        
        retries = 3
        self.start_audio_recording_srv(InfomaxAction.PLACE, self.category_id)
        
        for trial in range(retries):
            if self.move_arm_joint_goal(self.joint_names, joint_positions, link_padding=gripper_paddings, collision_operations=ordered_collision_operations):
                self.open_gripper()
                rospy.sleep(0.5)
                sound = self.stop_audio_recording_srv(True)
                resp = self.classification_srv(sound.recorded_sound)
                print 'PLACE RESULT'
                print resp
                
                obj = AttachedCollisionObject()
                obj.object.header.stamp = rospy.Time.now()
                obj.object.header.frame_id = 'L7_wrist_roll_link'
                obj.object.operation.operation = CollisionObjectOperation.DETACH_AND_ADD_AS_OBJECT
                obj.object.id = collision_object_name
                obj.link_name = 'L7_wrist_roll_link'
                obj.touch_links = ['L7_wrist_roll_link', 'L8_left_finger_link', 'L9_right_finger_link']
                
                self.attached_object_pub.publish(obj)
                
                rospy.sleep(2)
                return resp.beliefs
        else:
            self.stop_audio_recording_srv(False)
            rospy.logerr('failed to place arm after %d trials' % retries)
            return None

    def shake_roll(self, tabletop_collision_map_processing_result):
        desired_velocity = 11.0
        distance = 2.5
        threshold = 0.2
        self.wrist_roll_velocity_srv(2.0)
        self.wrist_roll_command_pub.publish(distance)
        
        while rospy.wait_for_message('wrist_roll_controller/state', DynamixelJointState).current_pos < distance-threshold:
            rospy.sleep(10e-3)
            
        self.wrist_roll_velocity_srv(desired_velocity)
        self.start_audio_recording_srv(InfomaxAction.SHAKE_ROLL, self.category_id)
        rospy.sleep(0.5)
        
        for i in range(2):
            self.wrist_roll_command_pub.publish(-distance)
            while rospy.wait_for_message('wrist_roll_controller/state', DynamixelJointState).current_pos > -distance+threshold:
                rospy.sleep(10e-3)
                
            self.wrist_roll_command_pub.publish(distance)
            while rospy.wait_for_message('wrist_roll_controller/state', DynamixelJointState).current_pos < distance-threshold:
                rospy.sleep(10e-3)
                
        rospy.sleep(0.5)
        sound = self.stop_audio_recording_srv(True)
        resp = self.classification_srv(sound.recorded_sound)
        print 'SHAKE_ROLL RESULT'
        print resp
        
        self.wrist_roll_velocity_srv(2.0)
        self.wrist_roll_command_pub.publish(0.0)
        
        rospy.sleep(2)
        return resp.beliefs

    def reset_robot(self, tabletop_collision_map_processing_result=None):
        if tabletop_collision_map_processing_result:
            obj = AttachedCollisionObject()
            obj.object.header.stamp = rospy.Time.now()
            obj.object.header.frame_id = 'L7_wrist_roll_link'
            obj.object.operation.operation = CollisionObjectOperation.REMOVE
            obj.object.id = tabletop_collision_map_processing_result.collision_object_names[0]
            obj.link_name = 'L7_wrist_roll_link'
            obj.touch_links = ['L7_wrist_roll_link', 'L8_left_finger_link', 'L9_right_finger_link']
            
            self.attached_object_pub.publish(obj)
            rospy.sleep(2)
            
        self.open_gripper()
        self.reset_collision_map()
        if not self.tuck_arm(): return False
        self.close_gripper()
        return True

    # receive an InfoMax service request containing object ID and desired action
    def process_infomax_request(self, req):
        self.object_names = req.objectNames
        self.action_names = req.actionNames
        self.num_categories = req.numCats
        self.category_id = req.catID
        
        prereqs = {InfomaxAction.GRASP:      [self.grasp],
                   InfomaxAction.LIFT:       [self.grasp, self.lift],
                   InfomaxAction.SHAKE_ROLL: [self.grasp, self.lift, self.shake_roll],
                   InfomaxAction.DROP:       [self.grasp, self.lift, self.drop],
                   InfomaxAction.PLACE:      [self.grasp, self.lift, self.place]}
                   
        if not self.reset_robot(): return None
        
        # find a graspable object on the floor
        tdr = self.segment_objects()
        if tdr is None: return None
        
        # mark floor and object poitions in the collision map as known
        tcmpr = self.update_collision_map(tdr)
        
        # move arm to ready position where gripper can be safely opened
        if not self.ready_arm(): return None
        
        # initialize as uniform distribution
        beliefs = [1.0/self.num_categories] * self.num_categories
        actions = prereqs[req.actionID.val]
        
        for act in actions:
            beliefs = act(tcmpr)
            
            if beliefs is None:
                self.reset_robot()
                return None
            
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

