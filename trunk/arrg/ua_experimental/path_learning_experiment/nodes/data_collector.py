#!/usr/bin/env python
# Author: Daniel Hewlett
# Author: Antons Rebguns

PKG = 'path_learning_experiment'

import roslib; roslib.load_manifest(PKG)
import rospy
import actionlib

from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, Pose, Point
from move_base_msgs.msg import MoveBaseActionFeedback
from actionlib_msgs.msg import GoalStatusArray
from nav_msgs.msg import Path
from path_learning_experiment.msg import DistanceInfo

import random
import math

import matplotlib.pyplot as plt

def dist(x1, y1, x2, y2):
       return math.sqrt(math.pow(x2-x1,2) + math.pow(y2-y1,2))

class Episode():
    def __init__(self, plan_waypoints, actual_waypoints, distances):
        #self.goal_x = goal_pos.position.x
        #self.goal_y = goal_pos.position.y
        #self.start_x = start_pos.position.x
        #self.start_y = start_pos.position.y
        
        # TODO: record episode status
        self.success = False
        self.planned_x = []
        self.planned_y = []
        
        for waypoint in plan_waypoints:
            self.planned_x += [waypoint.position.x]
            self.planned_y += [waypoint.position.y]
            
        # Hack, this should really come from the actual goal 
        self.goal_x = self.planned_x[-1]
        self.goal_y = self.planned_y[-1]
        
        self.dist_t = []
        self.dist_value = []
        self.start_time = actual_waypoints[0].header.stamp
        self.end_time = actual_waypoints[-1].header.stamp
        
        self.current_x = []
        self.current_y = []
        
        for waypoint in actual_waypoints:
            self.current_x += [waypoint.pose.position.x]
            self.current_y += [waypoint.pose.position.y]    
            self.dist_t += [(waypoint.header.stamp - self.start_time).to_sec()]
            self.dist_value += [dist(self.current_x[-1], self.current_y[-1], self.goal_x, self.goal_y)]
        
        self.scan_t = []
        self.scan_min = []
        
        for d in distances:
            if d.header.stamp >= self.start_time and d.header.stamp <= self.end_time:
                self.scan_t += [(d.header.stamp - self.start_time).to_sec()]
                self.scan_min += [d.min_dist]
                
    def plot(self):
        t = rospy.Time.now()
        
        plt.clf()
        plt.subplot(211)
        plt.plot(self.planned_x, self.planned_y, 'bo', label='Planned path')
        plt.plot(self.current_x, self.current_y, 'gv', label='Actual path')
        plt.axis([-3.5,3.5,-2.5,2.5])
        plt.xlabel('Distance from x origin [m]')
        plt.ylabel('Distance from y origin [m]')
        plt.title('Planned vs. actual path')
        plt.legend(loc=0)
#        plt.draw()
#        plt.savefig(str(t) + '-image.png')
        
#        plt.clf()
        plt.subplot(212)
        plt.plot(self.dist_t, self.dist_value, label='Distance to goal')
        plt.plot(self.scan_t, self.scan_min, 'r', label='Distance to obstacle')
        plt.axis(ymin=0.0, ymax=5.0)
        plt.xlabel('Time')
        plt.ylabel('Distance [m]')
        plt.legend(loc=0)
        
        plt.draw()
        plt.savefig(str(t) + '-plot.png')
        #plt.show()

class DataCollector():
    def __init__(self):
        self.x_range = (-3.5,3.5)
        self.y_range = (-2.5,2.5)
        
        self.previous_success = True
        self.episodes = []
        self.goal_pos = None
        self.plan_waypoints = []
        self.actual_waypoints = []
        self.distances = []
        
        rospy.init_node('data_collector', anonymous=True)
        
        rospy.Subscriber('move_base_simple/goal', PoseStamped, self.handle_goal)
        rospy.Subscriber('move_base/feedback', MoveBaseActionFeedback, self.handle_feedback)
        rospy.Subscriber('move_base/status', GoalStatusArray, self.handle_status)
        rospy.Subscriber('move_base_node/NavfnROS/plan', Path, self.handle_plan)
        rospy.Subscriber('dist_info', DistanceInfo, self.handle_dist)
        
    def save_episode(self):
        e = Episode(self.plan_waypoints, self.actual_waypoints, self.distances)
        self.episodes += [e]
        self.plan_waypoints = []
        self.actual_waypoints = []
        self.distances = []
        e.plot()
        
    def handle_goal(self, msg):
        self.goal_pos = msg.pose
        
    def handle_status(self, msg):
        if msg.status_list:
            curr_status = msg.status_list[0].status
            
    def handle_feedback(self, msg):
        self.actual_waypoints += [msg.feedback.base_position]
        
    def handle_plan(self, msg):
        if self.plan_waypoints:
            self.save_episode()
        for pose_stamped in msg.poses:
            self.plan_waypoints += [pose_stamped.pose]
        
    def handle_dist(self, msg):
        self.distances += [msg]

if __name__ == '__main__':
    try:
        collector = DataCollector()
        rospy.spin()
    except rospy.ROSInterruptException: pass

