#!/usr/bin/env python

# Author: Antons Rebguns

import roslib; roslib.load_manifest('w2_object_manipulation_launch')
import rospy

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

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point32
from sensor_msgs.msg import JointState

from kinematics_msgs.srv import GetKinematicSolverInfo
from kinematics_msgs.srv import GetPositionIK
from kinematics_msgs.srv import GetPositionIKRequest

from arm_navigation_msgs.msg import JointConstraint
from arm_navigation_msgs.msg import ArmNavigationErrorCodes
from arm_navigation_msgs.msg import AllowedContactSpecification
from arm_navigation_msgs.msg import LinkPadding
from arm_navigation_msgs.msg import OrderedCollisionOperations
from arm_navigation_msgs.msg import CollisionOperation

from arm_navigation_msgs.msg import MoveArmAction
from arm_navigation_msgs.msg import MoveArmGoal

from arm_navigation_msgs.msg import AttachedCollisionObject
from arm_navigation_msgs.msg import CollisionObjectOperation

from arm_navigation_msgs.msg import Shape


def find_ik_for_grasping_pose(pose_stamped):
    rospy.wait_for_service('wubble_left_arm_kinematics/get_ik_solver_info')
    get_solver_info_srv = rospy.ServiceProxy('/wubble_left_arm_kinematics/get_ik_solver_info', GetKinematicSolverInfo)
    rospy.loginfo('connected to wubble_left_arm_kinematics/get_ik_solver_info service')

    solver_info = get_solver_info_srv()
    arm_joints = solver_info.kinematic_solver_info.joint_names

    rospy.wait_for_service('wubble_left_arm_kinematics/get_ik')
    get_ik_srv = rospy.ServiceProxy('/wubble_left_arm_kinematics/get_ik', GetPositionIK)
    rospy.loginfo('connected to wubble_left_arm_kinematics/get_ik service')

    req = GetPositionIKRequest()
    req.timeout = rospy.Duration(5.0)
    req.ik_request.ik_link_name = 'L7_wrist_roll_link';
    req.ik_request.pose_stamped = pose_stamped

    current_state = rospy.wait_for_message('/joint_states', JointState)
    rospy.loginfo('recevied current joint states')
    
    req.ik_request.ik_seed_state.joint_state.name = arm_joints
    req.ik_request.ik_seed_state.joint_state.position = [current_state.position[current_state.name.index(j)] for j in arm_joints]
    
    rospy.loginfo('current joint positions are')
    rospy.loginfo('joint names: %s' % str(req.ik_request.ik_seed_state.joint_state.name))
    rospy.loginfo('joint state: %s' % str(req.ik_request.ik_seed_state.joint_state.position))

    ik_result = get_ik_srv(req)
    
    if ik_result.error_code.val == ArmNavigationErrorCodes.SUCCESS:
        rospy.loginfo('found IK solution for given grasping pose')
        rospy.loginfo('solution joints: %s' % str(ik_result.solution.joint_state.name))
        rospy.loginfo('solution angles: %s' % str(ik_result.solution.joint_state.position))
        return ik_result.solution
    else:
        rospy.logerr('Inverse kinematics failed')
        return None


def move_arm_to_grasping_joint_pose(joint_names, joint_positions, allowed_contacts=[], link_padding=[], collision_operations=OrderedCollisionOperations()):
    move_arm_client = SimpleActionClient('move_left_arm', MoveArmAction)
    move_arm_client.wait_for_server()
    rospy.loginfo('connected to move_left_arm action server')
    
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
    
    move_arm_client.send_goal(goal)
    finished_within_time = move_arm_client.wait_for_result(rospy.Duration(200.0))
    
    if not finished_within_time:
        move_arm_client.cancel_goal()
        rospy.loginfo('timed out trying to achieve a joint goal')
    else:
        state = move_arm_client.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo('action finished: %s' % str(state))
            return True
        else:
            rospy.loginfo('action failed: %s' % str(state))
            return False


if __name__ == '__main__':
    rospy.init_node('test_manipulation', anonymous=True)
    
    attached_object_pub = rospy.Publisher('/attached_collision_object', AttachedCollisionObject)
    
    rospy.wait_for_service('/tabletop_segmentation')
    tabletop_segmentation_srv = rospy.ServiceProxy('/tabletop_segmentation', TabletopSegmentation)
    rospy.loginfo('connected to tabletop_segmentation service')
    
    rospy.wait_for_service('/tabletop_collision_map_processing/tabletop_collision_map_processing'')
    collision_map_processing_srv = rospy.ServiceProxy('/tabletop_collision_map_processing/tabletop_collision_map_processing', TabletopCollisionMapProcessing)
    rospy.loginfo('connected to tabletop_collision_map_processing service')
    
    rospy.wait_for_service('/GraspPlanning')
    grasp_planning_srv = rospy.ServiceProxy('/GraspPlanning', GraspPlanning)
    rospy.loginfo('connected to GraspPlanning service')
    
    segmentation_result = tabletop_segmentation_srv()
    
    if segmentation_result.result != TabletopSegmentationResponse.SUCCESS or len(segmentation_result.clusters) == 0:
        rospy.logerr('TabletopSegmentation did not find any clusters')
        exit(1)
        
    rospy.loginfo('TabletopSegmentation found %d clusters, will update collision map now' % len(segmentation_result.clusters))
    
    tdr = TabletopDetectionResult()
    tdr.table = segmentation_result.table
    tdr.clusters = segmentation_result.clusters
    tdr.result = segmentation_result.result
    
    req = TabletopCollisionMapProcessingRequest()
    req.detection_result = tdr
    req.reset_collision_models = True
    req.reset_attached_models = True
    req.reset_static_map = False
    req.take_static_collision_map = False
    req.desired_frame = 'base_link'
    
    coll_map_res = collision_map_processing_srv(req)
    
    rospy.loginfo('collision_map update is done')
    rospy.loginfo('there are %d graspable objects %s, collision support surface name is "%s"' %
                  (len(coll_map_res.graspable_objects), coll_map_res.collision_object_names, coll_map_res.collision_support_surface_name))
                  
    req = GraspPlanningRequest()
    req.arm_name = 'left_arm'
    req.target = coll_map_res.graspable_objects[0]
    req.collision_object_name = coll_map_res.collision_object_names[0]
    req.collision_support_surface_name = coll_map_res.collision_support_surface_name
    
    rospy.loginfo('trying to find a good grasp for graspable object %s' % coll_map_res.collision_object_names[0])
    grasping_result = grasp_planning_srv(req)
    
    if grasping_result.error_code.value != GraspPlanningErrorCode.SUCCESS:
        rospy.logerr('unable to find a feasable grasp, aborting')
        exit(1)
        
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = grasping_result.grasps[0].grasp_posture.header.frame_id
    pose_stamped.pose = grasping_result.grasps[0].grasp_pose
    
    rospy.loginfo('found good grasp, looking for corresponding IK')
    ik_solution = find_ik_for_grasping_pose(pose_stamped)
    
    if ik_solution is None:
        exit(1)
        
    joint_names = ('shoulder_pitch_joint',
                   'shoulder_pan_joint',
                   'upperarm_roll_joint',
                   'elbow_flex_joint',
                   'forearm_roll_joint',
                   'wrist_pitch_joint',
                   'wrist_roll_joint')
                   
    # pose that should be executed before anything else to allow opening gripper
    joint_positions = (-1.760, -1.760, 0.659, -0.795, -2.521, 0.619, -0.148)
    if not move_arm_to_grasping_joint_pose(joint_names, joint_positions):
        exit(1)
        
    posture_controller = SimpleActionClient('wubble_gripper_grasp_action', GraspHandPostureExecutionAction)
    posture_controller.wait_for_server()
    rospy.loginfo('connected to gripper posture controller')
    
    pg = GraspHandPostureExecutionGoal()
    pg.goal = GraspHandPostureExecutionGoal.RELEASE
    
    posture_controller.send_goal(pg)
    posture_controller.wait_for_result()
    
    # define allowed contacts
    table = segmentation_result.table
    table_contact = AllowedContactSpecification()
    table_contact.name = coll_map_res.collision_support_surface_name
    table_contact.shape = Shape(type=Shape.BOX, dimensions=[abs(table.x_max - table.x_min), abs(table.y_max - table.y_min), 1e-6])
    table_contact.pose_stamped = table.pose
    table_contact.link_names = ['L9_right_finger_link', 'L8_left_finger_link', 'L7_wrist_roll_link', 'L6_wrist_pitch_link']
    table_contact.penetration_depth = 0.01
    allowed_contacts = []#[table_contact,]
    
    # define temporary link paddings
    gripper_paddings = [LinkPadding(l,0.0) for l in ('L9_right_finger_link', 'L8_left_finger_link')]
    
    if not move_arm_to_grasping_joint_pose(ik_solution.joint_state.name, ik_solution.joint_state.position, allowed_contacts, gripper_paddings):
        exit(1)
        
    rospy.sleep(1)
    
    pg = GraspHandPostureExecutionGoal()
    pg.goal = GraspHandPostureExecutionGoal.GRASP
    
    posture_controller.send_goal(pg)
    posture_controller.wait_for_result()
    
    rospy.sleep(1)
    
    obj = AttachedCollisionObject()
    obj.object.header.stamp = rospy.Time.now()
    obj.object.header.frame_id = 'L7_wrist_roll_link'#'base_link'
    obj.object.operation.operation = CollisionObjectOperation.ATTACH_AND_REMOVE_AS_OBJECT
    obj.object.id = coll_map_res.collision_object_names[0]
    obj.link_name = 'L7_wrist_roll_link'
    obj.touch_links = ['L7_wrist_roll_link', 'L8_left_finger_link', 'L9_right_finger_link']
    
    attached_object_pub.publish(obj)
    
    rospy.sleep(1)
    
    ordered_collision_operations = OrderedCollisionOperations()
    collision_operation = CollisionOperation()
    collision_operation.object1 = CollisionOperation.COLLISION_SET_ATTACHED_OBJECTS
    collision_operation.object2 = coll_map_res.collision_support_surface_name
    collision_operation.operation = CollisionOperation.DISABLE
    ordered_collision_operations.collision_operations = [collision_operation]
    
    if not move_arm_to_grasping_joint_pose(joint_names, joint_positions, allowed_contacts, gripper_paddings, ordered_collision_operations):
        exit(1)
        
    rospy.loginfo('Pickup stage has successfully finished. Will place the object now')
    
    ############################################################################
    ####################### PLACE STAGE START HERE #############################
    
    listener = TransformListener()
    
    # move grasped object and find a good grasp do we can approach and place
    obj_pcluster = listener.transformPointCloud('base_link', coll_map_res.graspable_objects[0].cluster)
    x = [point.x for point in obj_pcluster.points]
    y = [point.y for point in obj_pcluster.points]
    z = [point.z for point in obj_pcluster.points]
    offset = [0.0, -0.2, 0.0]
    
    req = GraspPlanningRequest()
    req.arm_name = 'left_arm'
    req.target.cluster.header.frame_id = 'base_link'
    req.target.cluster.points = [Point32(x[i]+offset[0], y[i]+offset[1], z[i]+offset[2]) for i in range(len(x))]
    req.target.type = GraspableObject.POINT_CLUSTER
    req.collision_object_name = coll_map_res.collision_object_names[0]
    req.collision_support_surface_name = coll_map_res.collision_support_surface_name
    
    rospy.loginfo('trying to find a good grasp for graspable object %s' % coll_map_res.collision_object_names[0])
    grasping_result = grasp_planning_srv(req)
    
    if grasping_result.error_code.value != GraspPlanningErrorCode.SUCCESS:
        rospy.logerr('unable to find a feasable grasp, aborting')
        exit(1)
        
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = grasping_result.grasps[0].grasp_posture.header.frame_id
    pose_stamped.pose = grasping_result.grasps[0].grasp_pose
    
    rospy.loginfo('found good grasp, looking for corresponding IK')
    ik_solution = find_ik_for_grasping_pose(pose_stamped)
    
    if ik_solution is None:
        exit(1)
        
    # move arm to target pose and disable collisions between the object an the table
    ordered_collision_operations = OrderedCollisionOperations()
    collision_operation = CollisionOperation()
    collision_operation.object1 = CollisionOperation.COLLISION_SET_ATTACHED_OBJECTS
    collision_operation.object2 = coll_map_res.collision_support_surface_name
    collision_operation.operation = CollisionOperation.DISABLE
    ordered_collision_operations.collision_operations = [collision_operation]
    
    if not move_arm_to_grasping_joint_pose(ik_solution.joint_state.name, ik_solution.joint_state.position, allowed_contacts, gripper_paddings, ordered_collision_operations):
        exit(1)
        
    rospy.sleep(1)
    
    pg = GraspHandPostureExecutionGoal()
    pg.goal = GraspHandPostureExecutionGoal.RELEASE
    
    posture_controller.send_goal(pg)
    posture_controller.wait_for_result()
    
    rospy.sleep(1)
    
    obj = AttachedCollisionObject()
    obj.object.header.stamp = rospy.Time.now()
    obj.object.header.frame_id = 'L7_wrist_roll_link'#'base_link'
    obj.object.operation.operation = CollisionObjectOperation.DETACH_AND_ADD_AS_OBJECT
    obj.object.id = coll_map_res.collision_object_names[0]
    obj.link_name = 'L7_wrist_roll_link'
    obj.touch_links = ['L7_wrist_roll_link', 'L8_left_finger_link', 'L9_right_finger_link']
    
    attached_object_pub.publish(obj)
    
    rospy.sleep(1)
    
    if not move_arm_to_grasping_joint_pose(joint_names, joint_positions):
        exit(1)
        
    rospy.sleep(1)
    
    pg = GraspHandPostureExecutionGoal()
    pg.goal = GraspHandPostureExecutionGoal.GRASP
    
    posture_controller.send_goal(pg)
    posture_controller.wait_for_result()
    
    ############################################################################
    
    rospy.spin()

