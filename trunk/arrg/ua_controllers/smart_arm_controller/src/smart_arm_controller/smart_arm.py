#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Arizona Robotics Research Group,
# University of Arizona. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of University of Arizona nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

import roslib
roslib.load_manifest('smart_arm_controller')

from ax12_driver_core.joint_position_controller import JointPositionControllerAX12
from ax12_driver_core.joint_position_controller_dual_motor import JointPositionControllerDualAX12

class DriverControl:
    def __init__(self, out_cb, joint_controllers):
        self.camera_pan_tilt = ArmAX12(out_cb, joint_controllers)
        
    def initialize(self):
        return self.camera_pan_tilt.initialize()
        
    def start(self):
        self.camera_pan_tilt.start()
        
    def stop(self):
        self.camera_pan_tilt.stop()

class ArmAX12():
    def __init__(self, out_cb, joint_controllers):
        self.shoulder_pan_controller = JointPositionControllerAX12(out_cb, joint_controllers[0])
        self.shoulder_tilt_controller = JointPositionControllerDualAX12(out_cb, joint_controllers[1])
        self.elbow_tilt_controller = JointPositionControllerDualAX12(out_cb, joint_controllers[2])
        self.wrist_rotate_controller = JointPositionControllerAX12(out_cb, joint_controllers[3])
        self.finger_right_controller = JointPositionControllerAX12(out_cb, joint_controllers[4])
        self.finger_left_controller = JointPositionControllerAX12(out_cb, joint_controllers[5])

    def initialize(self):
        success = (self.shoulder_pan_controller.initialize() and
                   self.shoulder_tilt_controller.initialize() and
                   self.elbow_tilt_controller.initialize() and
                   self.wrist_rotate_controller.initialize() and
                   self.finger_right_controller.initialize() and
                   self.finger_left_controller.initialize())

        if success:
            self.shoulder_pan_controller.set_speed(1.17)
            self.shoulder_tilt_controller.set_speed(1.17)
            self.elbow_tilt_controller.set_speed(1.17)
            self.wrist_rotate_controller.set_speed(1.17)
            self.finger_right_controller.set_speed(1.17)
            self.finger_left_controller.set_speed(1.17)

        return success
        
    def start(self):
        self.shoulder_pan_controller.start()
        self.shoulder_tilt_controller.start()
        self.elbow_tilt_controller.start()
        self.wrist_rotate_controller.start()
        self.finger_right_controller.start()
        self.finger_left_controller.start()
        
    def stop(self):
        self.shoulder_pan_controller.stop()
        self.shoulder_tilt_controller.stop()
        self.elbow_tilt_controller.stop()
        self.wrist_rotate_controller.stop()
        self.finger_right_controller.stop()
        self.finger_left_controller.stop()

