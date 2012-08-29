#!/usr/bin/env python

from random import random

import roslib
roslib.load_manifest('w2_object_manipulation_launch')

import rospy

from planar import Vec2, BoundingBox, Polygon

from bolt_representation.table2d.speaker import Speaker
from bolt_representation.table2d.landmark import PointRepresentation, RectangleRepresentation, Scene, Landmark, ObjectClass, Color
from bolt_representation.model2d import sentence_from_location

from bolt_msgs.msg import Scene as BoltScene
from bolt_msgs.srv import Correction
from bolt_msgs.srv import DescribePOI

from object_tracking.msg import ObjectCenters
import tf
from geometry_msgs.msg import PointStamped
import numpy as np
import cv

def point_to_vec(point):
    return Vec2(point.x, point.y)

def bolt_to_world(location):
    return Point32(location.y, -location.x, 0.0)

def world_to_bolt(point):
    return Vec2(-point.y, point.x)


class SceneConstructor(object):
    def __init__(self):
        self.tf = tf.TransformListener()
        rospy.Service('/describe_poi', DescribePOI, self.describe)
        rospy.Service('/correct_meaning', Correction, self.correct)
        self.scene_sub = rospy.Subscriber('bolt_scene', BoltScene, self.handle_scene)
        self.last_scene = None
        self.meaning = None
        self.speaker = Speaker(Vec2(0, 0))

    def describe(self, msg):
        if self.last_scene is not None:
            location = Landmark(msg.name, PointRepresentation( Vec2(msg.poi.x, msg.poi.y) ), None, Landmark.POINT)
            print self.speaker.describe(location, self.last_scene, msg.visualize, 1)
            
            self.meaning, sentence = sentence_from_location.generate_sentence( (msg.poi.x, msg.poi.y), False, self.last_scene, self.speaker )
            rospy.loginfo( 'Robot says: %s' % sentence )
            return sentence.encode('ascii', 'ignore')
        else:
            rospy.logerr('No scene to describe')
            return None

    def correct(self, msg):
        if self.meaning is not None:
            sentence_from_location.accept_correction(self.meaning, msg.correction)
            return []
        else:
            rospy.logerr('No meaning to correct')
            return None

    def handle_scene(self, msg):
        
        def get_image_polygon(obj):
            minps = PointStamped()
            minps.header.stamp = rospy.Time(0)
            minps.header.frame_id = 'base_footprint'
            minps.point.x = obj.points[0].y
            minps.point.y = -obj.points[0].x
            minps.point.z = obj.points[0].z
            
            minp = self.tf.transformPoint('kinect_rgb_optical_frame', minps).point
            
            minps.point.x = obj.points[1].y
            minps.point.y = -obj.points[1].x
            minps.point.z = obj.points[1].z
            
            maxp = self.tf.transformPoint('kinect_rgb_optical_frame', minps).point
            
            ct = np.array( [(minp.x,minp.y,minp.z), (maxp.x,minp.y,minp.z), (maxp.x,maxp.y,maxp.z), (minp.x,maxp.y,maxp.z)] )
            corners = cv.fromarray(ct)
            out = cv.fromarray(np.zeros((4,2)))
            cam_mat = np.array( [(525, 0, 319.5), (0, 525, 239.5), (0, 0, 1)] )
            cv.ProjectPoints2(corners,
                              cv.fromarray(np.zeros((1,3))),
                              cv.fromarray(np.zeros((1,3))),
                              cv.fromarray(cam_mat),
                              cv.fromarray(np.zeros((1,4))),
                              out)
            
            vs = []
            for l in np.asarray(out):
                vs.append( Vec2(*l) )
            
            return Polygon(vs)
        
        obj_centers = rospy.wait_for_message('/object_centers', ObjectCenters)
        
        scene = Scene(3)
        
        for obj,name in zip(msg.bboxes, msg.names):
            BB = BoundingBox([point_to_vec(obj.points[0]), point_to_vec(obj.points[1])])
            polygon = get_image_polygon(obj)
            if name == 'table':
                l = Landmark(name,
                             RectangleRepresentation( BB ),
                             None,
                             ObjectClass.TABLE)
            else:
                for center,color,category in zip(obj_centers.centers, obj_centers.color_labels, obj_centers.category_labels):
                    if polygon.contains_point( Vec2(center.pixel.x,center.pixel.y) ):
                        l = Landmark(name,
                                     RectangleRepresentation( BB, landmarks_to_get=[] ),
                                     None,
                                     category.upper(),
                                     color.upper())
                        l.representation.alt_representations= []
                        break
            scene.add_landmark(l)
            
        self.last_scene = scene
        rospy.loginfo('Constructed a scene')


if __name__ == '__main__':
    rospy.init_node('abstract_scene_constructor', anonymous=True)
    sc = SceneConstructor()
    rospy.loginfo('done')
    rospy.spin()
    
