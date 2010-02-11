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

# Run on Papak to start all ROS nodes and EvoCam for a WOZ Experiment

firefox -new-window "http://mini:8080/1/webcam.html" &
sleep 3
firefox -new-window "http://mini:8080/2/webcam.html" &
sleep 3

wmctrl -r "EvoCam 1" -e 1,0,25,350,375
wmctrl -r "EvoCam 2" -e 1,0,430,350,375

xterm -bg black -fg white -cr white -geometry 80x30+350+0 -e "rosrun woztools woztalker.py" &
xterm -bg black -fg white -cr white -geometry 80x30+835+0 -e "rosrun phidgets rfidlistener.py" &
xterm -bg black -fg white -cr white -geometry 80x30+350+450 -e "rosrun ccs robotcontrol.py" &
xterm -bg black -fg white -cr white -geometry 80x30+835+450 -e "rosrun woztools teachersubscriber.py" &


