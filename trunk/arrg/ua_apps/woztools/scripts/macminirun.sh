#!/usr/bin/env bash
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Arizona Robotics Research Group,
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

# Run on the Mac Mini to start all ROS nodes for a WOZ Experiment

xterm -bg black -fg white -cr white -geometry 80x20+350+0 -e "roscore" &
xterm -bg black -fg white -cr white -geometry 80x20+350+0 -e "rosrun woztools wozsubscriber.py" &
xterm -bg black -fg white -cr white -geometry 80x20+835+0 -e "rosrun ax12 init_sys.py /dev/tty.usbserial-A9005MZc; rosrun ax12 serialcomm.py" &
xterm -bg black -fg white -cr white -geometry 80x20+350+310 -e "rosrun ax12 movevalidator.py" &
xterm -bg black -fg white -cr white -geometry 80x20+1320+0 -e "rosrun ccs robot.py" &
xterm -bg black -fg white -cr white -geometry 80x20+835+310 -e "rosrun phidgets rfidscan.py" &
xterm -bg black -fg white -cr white -geometry 80x20+1320+310 -e "rosrun phidgets rfidlistener.py" &
xterm -bg black -fg white -cr white -geometry 100x30+750+598 -e "rosrun woztools wozlog.py" &