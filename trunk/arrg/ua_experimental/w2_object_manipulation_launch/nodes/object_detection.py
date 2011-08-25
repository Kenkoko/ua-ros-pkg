#!/usr/bin/env python

# Author: Antons Rebguns

import math
from operator import itemgetter

import roslib; roslib.load_manifest('w2_object_manipulation_launch')
import rospy

from tabletop_object_detector.msg import TabletopDetectionResult

from tabletop_object_detector.srv import TabletopSegmentation
from tabletop_object_detector.srv import TabletopSegmentationResponse

from object_manipulation_msgs.srv import FindClusterBoundingBox
from object_manipulation_msgs.srv import FindClusterBoundingBoxResponse

from visualization_msgs.msg import Marker

from std_msgs.msg import Float64
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Quaternion

from dynamixel_controllers.srv import SetSpeed
from dynamixel_msgs.msg import JointState

from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal

import tf

from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus

class ObjectDetector():
    def __init__(self, frame_id='L0_base_link',
                       min_dist=0.3,
                       max_dist=0.6,
                       min_ang=math.radians(-25),
                       max_ang=math.radians(25),
                       min_vol=0.0001,
                       max_vol=0.002):
        self.reference_frame = frame_id
        self.min_dist = min_dist
        self.max_dist = max_dist
        self.min_ang = min_ang
        self.max_ang = max_ang
        self.min_vol = min_vol
        self.max_vol = max_vol
        
        # connect to tabletop segmentation service
        rospy.loginfo('waiting for tabletop_segmentation service')
        rospy.wait_for_service('/tabletop_segmentation')
        self.tabletop_segmentation_srv = rospy.ServiceProxy('/tabletop_segmentation', TabletopSegmentation)
        rospy.loginfo('connected to tabletop_segmentation service')
        
        # connect to cluster bounding box finding service
        rospy.loginfo('waiting for find_cluster_bounding_box service')
        rospy.wait_for_service('/find_cluster_bounding_box')
        self.find_bounding_box_srv = rospy.ServiceProxy('/find_cluster_bounding_box', FindClusterBoundingBox)
        rospy.loginfo('connected to find_cluster_bounding_box')
        
        # connect to head speed setting services
        rospy.loginfo('waiting for head_tilt_controller/set_speed')
        rospy.wait_for_service('/head_tilt_controller/set_speed')
        self.set_head_tilt_speed = rospy.ServiceProxy('/head_tilt_controller/set_speed', SetSpeed)
        rospy.loginfo('connected to head_tilt_controller/set_speed')
        
        rospy.loginfo('waiting for head_pan_controller/set_speed')
        rospy.wait_for_service('/head_pan_controller/set_speed')
        self.set_head_pan_speed = rospy.ServiceProxy('/head_pan_controller/set_speed', SetSpeed)
        rospy.loginfo('connected to head_pan_controller/set_speed')
        
        rospy.loginfo('waiting for move_base/goal')
        self.move_base_client = SimpleActionClient('/move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        rospy.loginfo('connected to move_base/goal')
        
        self.head_tilt_pub = rospy.Publisher('/head_tilt_controller/command', Float64)
        self.head_pan_pub = rospy.Publisher('/head_pan_controller/command', Float64)
        
        self.current_head_tilt_state = rospy.wait_for_message('/head_tilt_controller/state', JointState)
        self.current_head_pan_state = rospy.wait_for_message('/head_pan_controller/state', JointState)
        
        self.head_tilt_state = rospy.Subscriber('/head_tilt_controller/state', JointState, self.process_head_tilt)
        self.head_tilt_state = rospy.Subscriber('/head_pan_controller/state', JointState, self.process_head_pan)
        
        self.marker_pub = rospy.Publisher('visualization_marker', Marker)
        
        self.head_speed = 0.5
        
        self.listener = tf.TransformListener()


    def process_head_tilt(self, msg):
        self.current_head_tilt_state = msg


    def process_head_pan(self, msg):
        self.current_head_pan_state = msg


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


    def point_head(self, loc='center'):
        self.set_head_pan_speed(self.head_speed)
        self.set_head_tilt_speed(self.head_speed)
        
        def wait_until_done():
            # wait for the command to get to the motors
            rospy.sleep(0.2)
            r = rospy.Rate(100)
            
            while (abs(self.current_head_tilt_state.error) > 2e-2 or abs(self.current_head_pan_state.error) > 2e-2) or \
                  (abs(self.current_head_tilt_state.velocity) > 1e-6 or abs(self.current_head_pan_state.velocity) > 1e-6):
                r.sleep()
                
        if loc == 'center':
            self.head_tilt_pub.publish(1.2)
            self.head_pan_pub.publish(0.0)
            wait_until_done()
        elif loc == 'left':
            self.head_tilt_pub.publish(1.2)
            self.head_pan_pub.publish(1.0)
            wait_until_done()
        elif loc == 'right':
            self.head_tilt_pub.publish(1.2)
            self.head_pan_pub.publish(-1.0)
            wait_until_done()


    def move_to_object(self, info):
        ang = info[2]
        if ang < math.radians(self.min_ang) or ang > math.radians(self.max_ang):
            print 'Turning to object'
            mbg = MoveBaseGoal()
            mbg.target_pose.header.stamp = rospy.Time.now()
            mbg.target_pose.header.frame_id = self.reference_frame
            ori = tf.transformations.quaternion_about_axis(ang, (0,0,1))
            mbg.target_pose.pose.orientation = Quaternion(*ori)
            
            self.move_base_client.send_goal(mbg)
            self.move_base_client.wait_for_result()
            
            state = self.move_base_client.get_state()
            
            if state == GoalStatus.SUCCEEDED:
                print 'yehhah!, we are looking straight at you, object'


    def within_limits(self, info):
        if (info[1] >= self.min_dist and info[1] <= self.max_dist) and \
           (info[3] >= self.min_vol and info[3] <= self.max_vol):
            return True
        else:
            return False


    def detect(self):
        obj_info = []
        
        # make sure the robot's head is pointing in the right direction
        self.point_head('center')
        
        # segment point cloud and get point clusters corresponding to objects on the floor
        seg_res = self.segment_objects()
        if seg_res is None: return None
        
        for idx,cluster in enumerate(seg_res.clusters):
            bbox_response = self.find_bounding_box_srv(cluster)
            
            if bbox_response.error_code != FindClusterBoundingBoxResponse.SUCCESS:
                rospy.logwarn('unable to find for cluster %d bounding box' % idx)
                continue
                
            # transform bounding box pose to another reference frame (arm base by default)
            bbox_response.pose.header.stamp = rospy.Time(0)
            bbox_pose_stamped = self.listener.transformPose(self.reference_frame, bbox_response.pose)
            
            bbox_pose = bbox_pose_stamped.pose
            bbox_dims = bbox_response.box_dims
            
            # create a bounding box visualization marker that's valid for 10 seconds
            marker = Marker()
            marker.header.frame_id = self.reference_frame
            marker.header.stamp = rospy.Time.now()
            marker.ns = 'floor_object_bboxes'
            marker.id = idx
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = bbox_pose
            marker.scale = bbox_dims
            marker.color = ColorRGBA(1.0, 0.0, 0.0, 0.5)
            marker.lifetime = rospy.Duration(10)
            
            # calculate 3D distance to cluster from the reference frame origin
            # if the object is behind the arm make sure we know about it
            dist = math.sqrt(bbox_pose.position.x**2 + bbox_pose.position.y**2 + bbox_pose.position.z**2)
            dist = math.copysign(dist, bbox_pose.position.x)
            
            # calculate angle to object
            ang = math.atan2(bbox_pose.position.y, bbox_pose.position.x)
            
            # calculate object bounding box volume
            vol = bbox_dims.x * bbox_dims.y * bbox_dims.z
            
            rospy.loginfo('[%d] dist = %.3f; ang = %.3f; vol = %f; pos = %s; bbox_dims = %s' % (idx, dist, math.degrees(ang), vol, str(bbox_pose.position).replace('\n',' '), str(bbox_dims).replace('\n',' ')))
            
            info = (idx, dist, ang, vol)
            
            if self.within_limits(info):
                obj_info.append(info)
                
                # change color to green for objects reachable from robot's current position
                marker.color.r = 0.0
                marker.color.g = 1.0
                self.marker_pub.publish(marker)
            else:
                rospy.loginfo('cluster %d outside of specified limits, skipping' % idx)
                
        rospy.loginfo('there are %d clusters after filtering' % len(obj_info))
        obj_info_sorted = sorted(obj_info, key=itemgetter(1))
        
        for idx,inf in enumerate(obj_info_sorted):
            dist = inf[1]
            ang = inf[2]
            rospy.loginfo('[%d] dist = %.3f; ang = %.3f' % (idx, dist, math.degrees(ang)))
            
        return seg_res, obj_info_sorted


if __name__ == '__main__':
    rospy.init_node('objects_filter_node', anonymous=True)
    od = ObjectDetector()
    od.detect()

