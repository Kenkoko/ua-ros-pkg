### ua\_drivers ###

This stack contains ROS drivers for various components of the Wubble Robot, specifically:
  * Drivers for the Dynamixel AX-12 servos ([ax12\_driver\_core](http://www.ros.org/wiki/ax12_driver_core)). These servos are used by the robot's SmartArm, tilt laser, and head pan/tilt.
  * A Python interface to Phidgets pressure sensors and RFID sensors  ([phidgets\_py\_api](http://www.ros.org/wiki/phidgets_py_api), [phidgets\_ros](http://www.ros.org/wiki/phidgets_ros)).
  * Two interfaces to the Videre STOC camera: The first ([videre\_stereo\_cam](http://www.ros.org/wiki/videre_stereo_cam)) uses standard FireWire libraries to get the left and right images from the camera. The second ([videre\_stoc](http://www.ros.org/wiki/videre_stoc)) relies on Videre's proprietary SVS software packages to pull the left image and disparity image from the STOC.

### ua\_controllers ###

This package contains controllers for the Wubble Robot, which generally depend on ua\_drivers for interfacing with the hardware. Thus, packages in this stack provide a higher level of abstraction for robot subsystems than ua\_drivers.
  * Controllers for the SmartArm ([smart\_arm\_controller](http://www.ros.org/wiki/smart_arm_controller)), tilting laser ([wubble\_laser\_tilt\_controller](http://www.ros.org/wiki/wubble_laser_tilt_controller)), and head pan/tilt ([wubble\_camera\_pan\_tilt\_controller](http://www.ros.org/wiki/wubble_camera_pan_tilt_controller)).
  * Actions interfaces for all of the above controllers ([wubble\_actions](http://www.ros.org/wiki/wubble_actions)).
  * Message types common to all controllers ([ua\_controller\_msgs](http://www.ros.org/wiki/ua_controller_msgs)).
  * A control system for a simple Bioloid robot used in some Wizard-of-Oz experiments at UA ([ccs](http://www.ros.org/wiki/ccs)).
  * An inverse kinematics (IK) solver for the SmartArm, created with OpenRAVE ([smart\_arm\_kinematics](http://www.ros.org/wiki/smart_arm_kinematics)).

### ua\_robots ###

This stack contains URDF description files, 3D models, and configuration parameters for the Wubble Robot. For a component-by-component description of the robot, see WubbleRobot.

### ua\_vision ###

Contains packages related to vision on UA robots, including saliency tracking based on the

  * A ROS wrapper for Nick's Machine Perception Toolkit ([nmpt](http://www.ros.org/wiki/nmpt)), a vision library providing algorithms for computing salient parts of an image. This information is used for saliency-based tracking ([saliency\_tracking](http://www.ros.org/wiki/saliency_tracking)).

### ua\_cognition ###

This stack will contain the cognitive architecture for the Wubble Robot. Currently, it contains a ROS wrapper for the GBBopen blackboard system ([gbbopen](http://www.ros.org/wiki/gbbopen)).

### ua\_apps ###

  * Wubble Robot Gmapping application ([wubble\_mapping](http://www.ros.org/wiki/wubble_mapping)).
  * Wubble Robot teleoperation ([wubble\_teleop](http://www.ros.org/wiki/wubble_teleop)).
  * Wubble Robot blocks stacking ([wubble\_blocks](http://www.ros.org/wiki/wubble_blocks)).

### wubble\_world ###

This stack contains world files for the Gazebo simulator (in [wubble\_environments](http://www.ros.org/wiki/wubble_environments)), intended as an environment for continuing the [Wubble World](http://www.wubbleworld.com) project.