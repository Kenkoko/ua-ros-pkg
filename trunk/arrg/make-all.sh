#!/bin/bash
rosmake $1 --rosdep-install --rosdep-yes --skip-blacklist ua_cognition ua_drivers ua_vision ua_apps ua_controllers ua_robots wubble_world ua_experimental ua_language wubble_navigation wubble_navigation_apps videre_erratic

