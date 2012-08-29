#!/usr/bin/env python

import roslib; roslib.load_manifest('w2_object_manipulation_launch')
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2
import cv
from bolt_msgs.msg import Scene, MoveIt
from bolt_msgs.srv import DescribePOI
from geometry_msgs.msg import PointStamped, Point, Point32
from geometry_msgs.msg import Polygon as PolygonMsg
import tf
import numpy as np
from planar import Vec2, Polygon
from w2_object_manipulation_launch.point_cloud2 import read_points
from w2_object_manipulation_launch.object_detection import ObjectDetector
from object_tracking.msg import ObjectCenters


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
        
        self.tf = tf.TransformListener()
        self.polygons = []
        self.bridge = CvBridge()
        self.object_detector = ObjectDetector()
        self.font = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, shear=0, thickness=1, lineType=cv.CV_AA)
        
        cv.NamedWindow('img')
        cv.SetMouseCallback('img', self.handle_click)
        
        rospy.Subscriber('camera/rgb/image_color', Image, self.process_image)
        rospy.Subscriber('bolt_scene', Scene, self.update_scene)
        rospy.Subscriber('object_centers', ObjectCenters, self.update_object_centers)
        
        self.describe_poi = rospy.ServiceProxy('describe_poi', DescribePOI)
        
        self.overlay_pub = rospy.Publisher('bolt_overlay', Image)
        self.move_it_pub = rospy.Publisher('move_it', MoveIt)

    def update_object_centers(self, msg):
        self.object_centers_msg = msg

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
            self.describe_poi(world_to_bolt(p.point), 'point', False)
            
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
        if event == cv.CV_EVENT_RBUTTONUP:
            self.trajector_location = None
            self.selected_obj_id = None
            self.object_detector.detect()
            self.trajector_location = None
            self.selected_obj_id = None
            self.object_detector.detect()

    def update_scene(self, msg):
        self.bboxes = msg.bboxes
        new_boxes = []
        new_polygons = []
        for obj,name in zip(msg.bboxes, msg.names):
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
                              
            temp = []
            vs = []
            for l in np.asarray(out):
                vs.append( Vec2(*l) )
                temp.append( tuple(map(int, l)) )
                
            new_polygons.append( Polygon(vs) )
            new_boxes.append(temp)
            
        self.polygons = new_polygons
        self.boxes = new_boxes
        
        # Calculate the linear correlation between x and z in kinect

    def process_image(self, msg):
        try:
            cvim = self.bridge.imgmsg_to_cv(msg, 'bgr8')
        except CvBridgeError as err:
            rospy.logerr(e)
            return
            
        if self.boxes: cv.PolyLine(cvim, self.boxes, True, (0,0,255))
        if self.selected_obj_id: cv.FillPoly(cvim, [self.boxes[self.selected_obj_id]], (0,0,255,0.5))
        
        if self.object_centers_msg:
            for center,color,category in zip(self.object_centers_msg.centers, self.object_centers_msg.color_labels, self.object_centers_msg.category_labels):
                cv.PutText(cvim, color + ' ' + category, (center.pixel.x,center.pixel.y), self.font, (0,0,255))
                
        if self.trajector_location:
            cv.Circle(cvim, self.trajector_location, 3, (255,0,0), thickness=2, lineType=cv.CV_AA, shift=0)
            cv.PutText(cvim, 'trajector', self.trajector_location, self.font, (255,0,0))
            
        cv.ShowImage('img', cvim)
        cv.WaitKey(10)

if __name__ == '__main__':
    rospy.init_node('bolt_gui', anonymous=True)
    
    bg = BoltGui()
    
    rospy.spin()
    cv.DestroyAllWindows()
