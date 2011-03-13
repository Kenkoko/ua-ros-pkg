#!/usr/bin/env python

import roslib; roslib.load_manifest('w2_object_manipulation_launch')
import rospy

from tabletop_object_detector.srv import TabletopSegmentation
from tabletop_object_detector.msg import TabletopDetectionResult
from tabletop_collision_map_processing.srv import TabletopCollisionMapProcessing
from tabletop_collision_map_processing.srv import TabletopCollisionMapProcessingRequest
from object_manipulation_msgs.msg import PickupAction
from object_manipulation_msgs.msg import PickupActionGoal
from object_manipulation_msgs.msg import PickupGoal
from object_manipulation_msgs.msg import GraspableObject
from actionlib import SimpleActionClient

if __name__ == '__main__':
    rospy.init_node('test_manipulation', anonymous=True)
    
    rospy.wait_for_service('/tabletop_segmentation')
    print 'got tabletop_segmentation'
    
    object_manipulator = SimpleActionClient('object_manipulator/object_manipulator_pickup', PickupAction)
    object_manipulator.wait_for_server()
    print 'got object_manipulator/object_manipulator_pickup'
    
    get_obj = rospy.ServiceProxy('/tabletop_segmentation', TabletopSegmentation)
    res = get_obj()
    
    if res.result != 4 or len(res.clusters) == 0:
        rospy.logerr('TabletopSegmentation did not find any clusters')
        exit(1)
        
    tdr = TabletopDetectionResult()
    tdr.table = res.table
    tdr.clusters = res.clusters
    tdr.result = res.result
    
    update_collision_map = rospy.ServiceProxy('/tabletop_collision_map_processing/tabletop_collision_map_processing', TabletopCollisionMapProcessing)
    req = TabletopCollisionMapProcessingRequest()
    req.detection_result = tdr
    req.reset_collision_models = True
    req.reset_attached_models = True
    req.reset_static_map = True
    req.take_static_collision_map = True
    req.desired_frame = 'base_link'
    
    coll_map_res = update_collision_map(req)
    
    rospy.loginfo('found %d clusters, grabbing first' % len(coll_map_res.graspable_objects))
    #obj_pcluster = res.clusters[0]
    
    req = PickupGoal()
    req.arm_name = 'left_arm'
    req. target = coll_map_res.graspable_objects[0]
    #req.target.type = GraspableObject.POINT_CLUSTER
    #req.target.cluster = obj_pcluster
    req.desired_approach_distance = 0.05
    req.min_approach_distance = 0.03
    req.lift.direction.header.frame_id = 'base_link'
    req.lift.direction.vector.x = 0
    req.lift.direction.vector.y = 0
    req.lift.direction.vector.z = 1
    req.lift.desired_distance = 0.03
    req.lift.min_distance = 0.02
    req.collision_object_name = coll_map_res.collision_object_names[0]
    req.collision_support_surface_name = coll_map_res.collision_support_surface_name
    req.use_reactive_execution = False
    req.use_reactive_lift = False
    
    object_manipulator.send_goal(req)
    object_manipulator.wait_for_result()


