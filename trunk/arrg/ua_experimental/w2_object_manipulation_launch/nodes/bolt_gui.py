#!/usr/bin/env python

import roslib; roslib.load_manifest('w2_object_manipulation_launch')

import numpy as np
from planar import Vec2, Polygon, BoundingBox

import rospy
import cv
import tf

from cv_bridge import (
    CvBridge,
    CvBridgeError
)

from w2_object_manipulation_launch.point_cloud2 import read_points
from w2_object_manipulation_launch.object_detection import ObjectDetector

from bolt_representation.table2d.speaker import Speaker

from bolt_representation.table2d.landmark import (
    PointRepresentation,
    RectangleRepresentation,
    Scene,
    Landmark,
    ObjectClass,
    Color
)

from bolt_representation.model2d import (
    sentence_from_location,
    correction_testing,
    location_from_sentence,
)

from sensor_msgs.msg import (
    Image,
    PointCloud2
)

from geometry_msgs.msg import (
    PointStamped,
    Point,
    Point32,
    Polygon as PolygonMsg
)

from bolt_msgs.msg import (
    Scene as SceneMsg,
    MoveIt
)

from bolt_msgs.srv import (
    DescribePOI,
    DescribePOIRequest,
    Correction,
    AutoCorrect,
    GetObjectFromSentence,
)

from object_tracking.msg import ObjectCenters


def point_to_vec(point):
    return Vec2(point.x, point.y)

def bolt_to_world(location):
    return Point32(location.y, -location.x, 0.0)

def world_to_bolt(point):
    return Vec2(-point.y, point.x)



def bolt_to_world(location, z=0.0):
    return Point32(location.y, -location.x, z)

def world_to_bolt(point):
    return Point(-point.y, point.x, point.z)


class BoltGui(object):
    def __init__(self):
        self.bboxes = None
        self.boxes = None
        self.selected_obj_id = None
        self.object_centers_msg = None
        self.trajector_location = None
        self.current_scene = None
        self.meaning = None
        self.scene_lmk_to_bbox = {}
        
        self.speaker = Speaker(Vec2(0, 0))
        self.tf = tf.TransformListener()
        self.polygons = []
        self.bridge = CvBridge()
        self.object_detector = ObjectDetector()
        self.font = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, shear=0, thickness=1, lineType=cv.CV_AA)
        
        cv.NamedWindow('img')
        cv.SetMouseCallback('img', self.handle_click)
        
        rospy.Subscriber('camera/rgb/image_color', Image, self.process_image)
        rospy.Subscriber('object_centers', ObjectCenters, self.update_object_centers)
        
        rospy.Service('describe_poi', DescribePOI, self.describe)
        rospy.Service('correct_meaning', Correction, self.correct)
        rospy.Service('autocorrect_meaning', AutoCorrect, self.autocorrect)
        rospy.Service('get_object_from_sentence', GetObjectFromSentence, self.get_object_from_sentence)
        
        self.scene_pub = rospy.Publisher('bolt_scene', SceneMsg)
        self.overlay_pub = rospy.Publisher('bolt_overlay', Image)
        self.move_it_pub = rospy.Publisher('move_it', MoveIt)

    def get_object_from_sentence(self, msg):
        lmks = location_from_sentence.get_most_likely_object(self.current_scene, self.speaker, msg.sentences)
        return zip(*[(lmk.name,lmk.object_class,lmk.color) for lmk in lmks])

    def lmk_geo_to_img_geo(self, lmk):
        parent = lmk.get_top_parent().parent_landmark
        z = self.scene_lmk_to_bbox[parent].points[0].z
        
        lmk_points = lmk.representation.get_points()
        ct = np.zeros( (len(lmk_points),3) )
        
        for i,p in enumerate(lmk_points):
            ps = PointStamped()
            ps.header.stamp = rospy.Time(0)
            ps.header.frame_id = 'base_footprint'
            ps.point.x = p.y
            ps.point.y = -p.x
            ps.point.z = z
            
            pst = self.tf.transformPoint('kinect_rgb_optical_frame', ps).point
            ct[i] = (pst.x, pst.y, pst.z)
            
        corners = cv.fromarray(ct)
        out = cv.fromarray(np.zeros((len(lmk_points),2)))
        cam_mat = np.array( [(525, 0, 319.5), (0, 525, 239.5), (0, 0, 1)] )
        cv.ProjectPoints2(corners,
                          cv.fromarray(np.zeros((1,3))),
                          cv.fromarray(np.zeros((1,3))),
                          cv.fromarray(cam_mat),
                          cv.fromarray(np.zeros((1,4))),
                          out)
                          
#        print lmk, lmk_points, np.asarray(out)
        return np.asarray(out)

    def get_image_polygon(self, obj, name=''):
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
                          
        img_box = []
        vs = []
        for l in np.asarray(out):
            vs.append( Vec2(*l) )
            img_box.append( tuple(map(int, l)) )
            
        return Polygon(vs), img_box

    def update_object_centers(self, msg):
        self.object_centers_msg = msg

    def describe(self, msg):
        if self.current_scene is not None:
            location = Landmark(msg.name, PointRepresentation( Vec2(msg.poi.x, msg.poi.y) ), None, Landmark.POINT)
            print self.speaker.describe(location, self.current_scene, msg.visualize, 1)
            
            self.meaning, sentence = sentence_from_location.generate_sentence( (msg.poi.x, msg.poi.y), False, self.current_scene, self.speaker )
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

    def autocorrect(self, msg):
        correction_testing.autocorrect(self.current_scene, self.speaker, msg.num_iterations, msg.window, msg.scale)
        return []

    def construct_scene_msg(self, segmentation_result, cluster_information):
        scene = SceneMsg()
        
        # table
        table = segmentation_result.table
        t_center = table.pose.pose.position
        
        t_min = Point32(-(t_center.y + table.y_max), # x in robot space is -y is planar
                        t_center.x + table.x_min,
                        t_center.z)
                        
        t_max = Point32(-(t_center.y + table.y_min),
                        t_center.x + table.x_max,
                        t_center.z)
                        
        scene.bboxes.append(PolygonMsg([t_min, t_max]))
        scene.names.append('table')
        scene.colors.append('')
        scene.categories.append('')
        
        for idx,cluster in enumerate(segmentation_result.clusters):
            # find cluster with index idx and grab its bounding box
            #print cluster_information
            #print idx
            bbox = cluster_information[ [ci[0] for ci in cluster_information].index(idx) ][4]
            
            o_center = bbox.pose.pose.position
            o_min = Point32(-(o_center.y + bbox.box_dims.y/2.0),
                            o_center.x - bbox.box_dims.x/2.0,
                            o_center.z)
                            
            o_max = Point32(-(o_center.y - bbox.box_dims.y/2.0),
                            o_center.x + bbox.box_dims.x/2.0,
                            o_center.z)
                            
            scene.bboxes.append(PolygonMsg([o_min, o_max]))
            scene.names.append(str(idx))
            scene.colors.append('')
            scene.categories.append('')
            
        return scene

    def construct_abstract_scene(self, scene_msg):
        scene = Scene(3)
        
        for i,(obj,name) in enumerate(zip(scene_msg.bboxes, scene_msg.names)):
            BB = BoundingBox([point_to_vec(obj.points[0]), point_to_vec(obj.points[1])])
            polygon, _ = self.get_image_polygon(obj)
            
            if name == 'table':
                l = Landmark(name,
                             RectangleRepresentation( BB ),
                             None,
                             ObjectClass.TABLE)
            else:
                for center,color,category in zip(self.object_centers_msg.centers, self.object_centers_msg.color_labels, self.object_centers_msg.category_labels):
                    if polygon.contains_point( Vec2(center.pixel.x, center.pixel.y) ):
                        l = Landmark(name,
                                     RectangleRepresentation( BB, landmarks_to_get=[] ),
                                     None,
                                     category.upper(),
                                     color.upper())
                        l.representation.alt_representations= []
                        
                        scene_msg.colors[i] = color.upper()
                        scene_msg.categories[i] = category.upper()
                        
                        break
                        
            scene.add_landmark(l)
            self.scene_lmk_to_bbox[l] = obj
            
        self.scene_pub.publish(scene_msg)
        rospy.loginfo('Constructed a scene')
        return scene

    def handle_click(self, event, x, y, flags, params):
        if event == cv.CV_EVENT_LBUTTONUP:
            #print x, y
            self.trajector_location = (x,y)
            
            cloud = rospy.wait_for_message('/camera/rgb/points', PointCloud2)
            p = read_points(cloud, uvs=[(x,y)]).next()
            ps = PointStamped()
            ps.header.stamp = rospy.Time(0)
            ps.header.frame_id = cloud.header.frame_id
            ps.point = Point(*p[:-1])
            p = self.tf.transformPoint('base_footprint', ps)
            #print p
            #print world_to_bolt(p.point)
            self.describe( DescribePOIRequest(world_to_bolt(p.point), 'point', False) )
            
            if self.selected_obj_id:
                msg = MoveIt()
                msg.target_location = p.point
                bolt_box = self.bboxes[self.selected_obj_id]
                msg.object_bbox = PolygonMsg( [bolt_to_world(bolt_box.points[0]), bolt_to_world(bolt_box.points[1])] )
                print msg
                self.move_it_pub.publish(msg)
                self.selected_obj_id = None
                
            for i,p in enumerate(self.polygons):
                if p.contains_point( Vec2(x,y) ):
                    self.selected_obj_id = i
                    # todo: select minimum size rectangle
                    #break
                    
        elif event == cv.CV_EVENT_RBUTTONUP:
#            self.trajector_location = None
#            self.selected_obj_id = None
#            self.object_detector.detect()
            
            self.trajector_location = None
            self.selected_obj_id = None
            self.meaning = None
            self.scene_lmk_to_bbox = {}
            res = self.object_detector.detect()
            scene_msg = self.construct_scene_msg(res['segmentation_result'], res['cluster_information'])
            self.current_scene = self.construct_abstract_scene(scene_msg)
            self.update_scene(scene_msg)

    def update_scene(self, msg):
        self.bboxes = msg.bboxes
        new_boxes = []
        new_polygons = []
        
        for obj,name in zip(msg.bboxes, msg.names):
            poly,box = self.get_image_polygon(obj,name)
            new_polygons.append(poly)
            new_boxes.append(box)
            
        self.polygons = new_polygons
        self.boxes = new_boxes

    def process_image(self, msg):
        try:
            cvim = self.bridge.imgmsg_to_cv(msg, 'bgr8')
        except CvBridgeError as err:
            rospy.logerr(e)
            return
            
        if self.boxes: cv.PolyLine(cvim, self.boxes, True, (0,0,255))
        if self.selected_obj_id: cv.FillPoly(cvim, [self.boxes[self.selected_obj_id]], (0,0,255,0.5))
        
        # display object names
        if self.object_centers_msg:
            for center,color,category in zip(self.object_centers_msg.centers, self.object_centers_msg.color_labels, self.object_centers_msg.category_labels):
                cv.PutText(cvim, color + ' ' + category, (center.pixel.x,center.pixel.y), self.font, (0,0,255))
                
        # display trajector
        if self.trajector_location:
            cv.Circle(cvim, self.trajector_location, 3, (255,0,0), thickness=2, lineType=cv.CV_AA, shift=0)
            cv.PutText(cvim, 'trajector', self.trajector_location, self.font, (255,0,0))
            
        # display selected landmark
        if self.meaning:
            lmk = self.meaning.args[0]
            
            p_arr = self.lmk_geo_to_img_geo(lmk)
            
            if isinstance(lmk.representation, PointRepresentation):
                cv.Circle(cvim, tuple(map(int, p_arr[0])), 3, (0,255,0), thickness=2, lineType=cv.CV_AA, shift=0)
            else:
                pl = [[tuple(map(int, l)) for l in p_arr]]
                cv.PolyLine(cvim, pl, True, (0,255,0), thickness=3)
                
        cv.ShowImage('img', cvim)
        cv.WaitKey(10)


if __name__ == '__main__':
    rospy.init_node('bolt_gui', anonymous=True)
    
    bg = BoltGui()
    
    rospy.spin()
    cv.DestroyAllWindows()
    
