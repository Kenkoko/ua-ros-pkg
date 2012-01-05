#!/usr/bin/env python

import roslib; roslib.load_manifest('knowrob_tutorial')
import rospy

from tabletop_object_detector.srv import TabletopDetection
from tabletop_object_detector.msg import TabletopDetectionResult
from tabletop_object_detector.srv import TabletopDetectionResponse

from tabletop_object_detector.srv import TabletopSegmentation
from tabletop_object_detector.srv import TabletopSegmentationResponse

from object_manipulation_msgs.srv import FindClusterBoundingBox
from household_objects_database_msgs.msg import DatabaseModelPose
from household_objects_database_msgs.msg import DatabaseModelPoseList
from point_cloud_classifier.srv import GetClusterLabels
from point_cloud_classifier.srv import GetClusterLabelsRequest

class ObjectSegmentation:
    def __init__(self):
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
        
        rospy.loginfo('waiting for /get_cluster_labels')
        rospy.wait_for_service('/get_cluster_labels')
        self.classify_srv = rospy.ServiceProxy('/get_cluster_labels', GetClusterLabels)
        rospy.loginfo('connected to get_cluster_labels')
        
        rospy.Service('object_detection', TabletopDetection, self.segment_objects)

    def segment_objects(self, req):
        segmentation_result = self.tabletop_segmentation_srv()
        
        if segmentation_result.result != TabletopSegmentationResponse.SUCCESS or not segmentation_result.clusters:
            rospy.logerr('TabletopSegmentation did not find any clusters')
            return None
            
        rospy.loginfo('TabletopSegmentation found %d clusters' % len(segmentation_result.clusters))
        
        tdr = TabletopDetectionResult()
        tdr.table = segmentation_result.table
        tdr.clusters = segmentation_result.clusters
        tdr.result = segmentation_result.result
        
        req = GetClusterLabelsRequest()
        req.clusters = segmentation_result.clusters
        res = self.classify_srv(req)
        
        labels = res.labels
        
        index_to_label = {
            0: 'Lego',
            1: 'Box',
            2: 'Toy',
            3: 'Ball',
            4: 'Cup',
        }
        
        label_to_index = {
            'Lego': 0,
            'Box': 1,
            'Toy': 2,
            'Ball': 3,
            'Cup': 4,
        }
        
        for idx,cluster in enumerate(segmentation_result.clusters):
            bbox_response = self.find_bounding_box_srv(cluster)
            
            dmp = DatabaseModelPose()
            dmp.model_id = label_to_index[labels[idx]]
            dmp.pose = bbox_response.pose
            
            dmpl = DatabaseModelPoseList()
            dmpl.model_list = [dmp]
            
            tdr.models.append(dmpl)
            
        res = TabletopDetectionResponse()
        res.detection = tdr
        rospy.loginfo(res)
        
        return res


if __name__ == '__main__':
    rospy.init_node('object_detection_classification', anonymous=True)
    os = ObjectSegmentation()
    rospy.spin()

