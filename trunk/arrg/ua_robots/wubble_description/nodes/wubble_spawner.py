#! /usr/bin/env python

# Copyright (c) 2010, Arizona Robotics Research Group, University of Arizona
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Daniel Hewlett

PKG = 'wubble_description'
NAME = 'wubble_spawner'

import roslib; roslib.load_manifest(PKG)
import rospy

from tf.transformations import quaternion_about_axis

from geometry_msgs.msg import Quaternion
from wubble_description.srv import *
from gazebo.srv import *
from pr2_mechanism_msgs.srv import *
from std_srvs.srv import *

class WubbleSpawner():

    def __init__(self):
        rospy.init_node(NAME, anonymous=True)

        self.controllers = ['laser_tilt_controller']

        self.pause = switch_controller = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        self.unpause = switch_controller = rospy.ServiceProxy('gazebo/unpause_physics', Empty)

        # Spawn the wubble base model and start all of its controllers
        rospy.Service('spawn_wubble_base', SpawnWubbleBase, self.spawn_wubble_base)
        # Delete the wubble base model and stop all of its controllers
        rospy.Service('delete_wubble_base', DeleteWubbleBase, self.delete_wubble_base)

        rospy.loginfo("Services spawn_wubble_base and delete_wubble_base are ready.")

    def spawn_wubble_base(self, request):
        rospy.wait_for_service('/gazebo/spawn_urdf_model')
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
    
        model_req = SpawnModelRequest()
        model_req.reference_frame = "/map" # do we really need this?        
        model_req.model_name = request.name
        # where should we get this? from file or parameter?            
        model_req.model_xml = rospy.get_param("/robot_description")         
        model_req.robot_namespace = "/" 
        model_req.initial_pose = request.initial_pose
#        model_req.initial_pose.position = request.position # I think this is safe
#        q = quaternion_about_axis(request.orientation, (0, 0, 1)) 
#        model_req.initial_pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    
        spawn_result = spawn_urdf(model_req)
        
        if not spawn_result.success:
            return None

        # Now need to start the controllers
        rospy.wait_for_service('pr2_controller_manager/load_controller')
        load_controller = rospy.ServiceProxy('pr2_controller_manager/load_controller', LoadController)
        rospy.wait_for_service('pr2_controller_manager/switch_controller')
        switch_controller = rospy.ServiceProxy('pr2_controller_manager/switch_controller', SwitchController)
    
        self.unpause()

        # For now, just hard-coding the controller names
        # For this to work, the yaml files for the controller have to already be uploaded
        for controller in self.controllers:        
            load_controller(controller)
        switch_controller(self.controllers, [], 2)

        self.pause()

        return SpawnWubbleBaseResponse()

    def delete_wubble_base(self, request):
        self.unpause()

        # First, stop and then unload the controllers
        rospy.wait_for_service('pr2_controller_manager/unload_controller')
        unload_controller = rospy.ServiceProxy('pr2_controller_manager/unload_controller', LoadController)
        rospy.wait_for_service('pr2_controller_manager/switch_controller')
        switch_controller = rospy.ServiceProxy('pr2_controller_manager/switch_controller', SwitchController)

        switch_controller([], self.controllers, 2)
        for controller in self.controllers:
            unload_controller(controller)

        self.pause()
    
        # Now, delete the model
        rospy.wait_for_service('/gazebo/delete_model')
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

        # Probably can shorten this
        dmr = DeleteModelRequest(model_name=request.name)
        result = delete_model(dmr)
        
        if not result.success:
            return None
    
        return DeleteWubbleBaseResponse()

if __name__ == '__main__':
    try:
        w = WubbleSpawner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass




