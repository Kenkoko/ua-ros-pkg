# Overview #

This repository provides a number of packages that support the Wubble Robot created by the Arizona Robotics Research Group. Because a number of the components of the Wubble Robot are also used by other research groups, we provide a description of the robot and of relevant software packages. The [ua\_robots](http://www.ros.org/wiki/ua_robots) stack contains URDF description files for the Wubble Robot and its components. This URDF is linked to detailed 3D models of the robot's components, so that all of the robot's sensors and motors can be accurately simulated in Gazebo (seen below) and visualized in rviz, with correct transform information for tf.

# Wubble Robot #

<div>
<img src='http://ua-ros-pkg.googlecode.com/svn/trunk/arrg/wiki/images/wubble1-sim.png' alt='logo' /><img src='http://ua-ros-pkg.googlecode.com/svn/trunk/arrg/wiki/images/wubble1-rl.png' alt='background' />
</div>

Packages:
  * [wubble\_description](http://www.ros.org/wiki/wubble_description)
  * [wubble\_actions](http://www.ros.org/wiki/wubble_actions)
  * [wubble\_mapping](http://www.ros.org/wiki/wubble_mapping)
  * [wubble\_teleop](http://www.ros.org/wiki/wubble_teleop)
  * [wubble\_blocks](http://www.ros.org/wiki/wubble_blocks)

# Components #

## Videre ERRATIC (Mobile Base) ##

<img src='http://www.videredesign.com/assets/images/robot_images/robot/era-mobi_sm.png' height='100' />

[Description](http://www.videredesign.com/index.php?id=45)

Packages:
  * [erratic\_description](http://www.ros.org/wiki/erratic_description)
  * [ua\_erratic\_player](http://www.ros.org/wiki/ua_erratic_player)
  * [wubble\_plugins](http://www.ros.org/wiki/wubble_plugins)

## Videre STOC (Stereo Camera) ##

<img src='http://www.videredesign.com/assets/images/vision_images/products/stoc-sthdcsg-9cm_med.png' height='100' />

[Description](http://www.videredesign.com/index.php?id=74)

Packages:
  * [videre\_stoc\_description](http://www.ros.org/wiki/videre_stoc_description)
  * [videre\_stereo\_cam](http://www.ros.org/wiki/videre_stereo_cam)
  * [videre\_stoc](http://www.ros.org/wiki/videre_stoc)
  * [stoc\_publisher](http://www.ros.org/wiki/stoc_publisher)

## Hokuyo URG (Laser Range Finder) ##

<img src='http://www.acroname.com/robotics/parts/R325-URG-04LX-UG01.jpg' height='100' />

[Description](http://www.acroname.com/robotics/parts/R325-URG-04LX-UG01.html)

Packages:
  * [houkyo\_urg\_description](http://www.ros.org/wiki/houkyo_urg_description)

## Crust Crawler Smart Arm ##

<img src='http://www.crustcrawler.com/products/smartarm/images/bigGrip2FingerLarge.jpg' height='100' />

[Description](http://www.crustcrawler.com/products/smartarm/index.php?prod=12#thumb)

Packages:
  * [smart\_arm\_description](http://www.ros.org/wiki/smart_arm_description)
  * [smart\_arm\_controller](http://www.ros.org/wiki/smart_arm_controller)
  * [smart\_arm\_kinematics](http://www.ros.org/wiki/smart_arm_kinematics)

## Robotis AX-12+ Servos ##

<img src='http://www.crustcrawler.com/motors/AX12/images/AX_12.jpg' height='100' />

[Description](http://www.robotis.com/zbxe/dynamixel_en)

Packages:
  * [ax12\_driver\_core](http://www.ros.org/wiki/ax12_driver_core)
  * [ax12\_controller\_core](http://www.ros.org/wiki/ax12_controller_core)

## Prosilica GC1600CH GigE Vision camera ##

<img src='http://www.prosilica.com/images/cameras/GC_back_400x254.jpg' height='100' />

[Description](http://www.prosilica.com/products/gc1600h.html)

Packages:
  * [prosilica\_gige\_sdk](http://www.ros.org/wiki/prosilica_gige_sdk)
  * [prosilica\_camera](http://www.ros.org/wiki/prosilica_camera)
  * [ua\_overhead\_cam](http://www.ros.org/browse/details.php?name=ua_overhead_cam)
  * [background\_filters](http://www.ros.org/browse/details.php?name=background_filters)