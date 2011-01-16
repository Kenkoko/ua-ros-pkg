(in-package :simsem)

(defparameter *robot-attributes* #("x" "y" "orientation" "last_x" "last_y" "last_orientation"))
(defparameter *location-attributes* #("x" "y"))
(defparameter *object-attributes* #("x" "y" "orientation" "last_x" "last_y" "last_orientation"
                                    "size_x" "size_y" "size_z" "mass" "static" "shape" "color"))
(defparameter *item-attributes* #("x" "y" "last_x" "last_y" "is_carried" "carrier_name"))

(defparameter *gazebo-go-training* 
  (list 
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "robot_description"
                           :class_name "Robot"
                           :attributes *robot-attributes*
                           :values #("0.0" "0.0" "NE" "0.0" "0.0" "NE"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "goal"
                           :class_name "Location"
                           :attributes *location-attributes*
                           :values #("1.0" "0.5")))
         '("forward" "right" "forward"))
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "robot_description"
                           :class_name "Robot"
                           :attributes *robot-attributes*
                           :values #("0.0" "0.0" "W" "0.0" "0.0" "W"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "goal"
                           :class_name "Location"
                           :attributes *location-attributes*
                           :values #("-1.5" "0.0")))
         '("forward" "forward" "forward"))
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "robot_description"
                           :class_name "Robot"
                           :attributes *robot-attributes*
                           :values #("0.0" "0.0" "SE" "0.0" "0.0" "SE"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "goal"
                           :class_name "Location"
                           :attributes *location-attributes*
                           :values #("0.5" "0.0")))
         '("left" "forward"))
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "robot_description"
                           :class_name "Robot"
                           :attributes *robot-attributes*
                           :values #("0.0" "0.0" "E" "0.0" "0.0" "E"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "goal"
                           :class_name "Location"
                           :attributes *location-attributes*
                           :values #("0.5" "0.0")))
         '("forward"))
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "robot_description"
                           :class_name "Robot"
                           :attributes *robot-attributes*
                           :values #("0.0" "0.0" "N" "0.0" "0.0" "N"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "goal"
                           :class_name "Location"
                           :attributes *location-attributes*
                           :values #("0.0" "-1.0")))
         '("left" "left" "left" "left" "forward" "forward"))
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "robot_description"
                           :class_name "Robot"
                           :attributes *robot-attributes*
                           :values #("0.0" "0.0" "N" "0.0" "0.0" "N"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "goal"
                           :class_name "Location"
                           :attributes *location-attributes*
                           :values #("1.0" "0.0")))
         '("right" "right" "forward" "forward"))
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "robot_description"
                           :class_name "Robot"
                           :attributes *robot-attributes*
                           :values #("0.0" "0.0" "N" "0.0" "0.0" "N"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "goal"
                           :class_name "Location"
                           :attributes *location-attributes*
                           :values #("0.0" "3.0")))
         '("forward" "forward" "forward" "forward" "forward" "forward"))
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "robot_description"
                           :class_name "Robot"
                           :attributes *robot-attributes*
                           :values #("0.0" "0.0" "N" "0.0" "0.0" "N"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "goal"
                           :class_name "Location"
                           :attributes *location-attributes*
                           :values #("0.0" "-1.0")))
         '("right" "right" "right" "right" "forward" "forward"))
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "robot_description"
                           :class_name "Robot"
                           :attributes *robot-attributes*
                           :values #("0.0" "0.0" "N" "0.0" "0.0" "N"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "goal"
                           :class_name "Location"
                           :attributes *location-attributes*
                           :values #("-0.5" "1.0")))
         '("forward" "left" "forward"))
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "robot_description"
                           :class_name "Robot"
                           :attributes *robot-attributes*
                           :values #("0.0" "0.0" "N" "0.0" "0.0" "N"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "goal"
                           :class_name "Location"
                           :attributes *location-attributes*
                           :values #("1.0" "1.5")))
         '("right" "forward" "forward" "left" "forward"))
   ))

(defparameter *gazebo-go-through-training*
  (list 
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "robot_description"
                           :class_name "Robot"
                           :attributes *robot-attributes*
                           :values #("0.0" "0.0" "E" "0.0" "0.0" "E"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "waypoint"
                           :class_name "Location"
                           :attributes *location-attributes*
                           :values #("0.5" "0.0"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "goal"
                           :class_name "Location"
                           :attributes *location-attributes*
                           :values #("1.0" "0.0")))
         '("forward" "forward"))
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "robot_description"
                           :class_name "Robot"
                           :attributes *robot-attributes*
                           :values #("0.0" "0.0" "N" "0.0" "0.0" "N"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "waypoint"
                           :class_name "Location"
                           :attributes *location-attributes*
                           :values #("0.0" "1.0"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "goal"
                           :class_name "Location"
                           :attributes *location-attributes*
                           :values #("0.0" "2.0")))
         '("forward" "forward" "forward" "forward"))
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "robot_description"
                           :class_name "Robot"
                           :attributes *robot-attributes*
                           :values #("0.0" "0.0" "E" "0.0" "0.0" "E"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "waypoint"
                           :class_name "Location"
                           :attributes *location-attributes*
                           :values #("0.5" "0.0"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "goal"
                           :class_name "Location"
                           :attributes *location-attributes*
                           :values #("1.5" "0.0")))
         '("forward" "forward" "forward"))
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "robot_description"
                           :class_name "Robot"
                           :attributes *robot-attributes*
                           :values #("0.0" "0.0" "W" "0.0" "0.0" "W"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "waypoint"
                           :class_name "Location"
                           :attributes *location-attributes*
                           :values #("-1.0" "0.0"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "goal"
                           :class_name "Location"
                           :attributes *location-attributes*
                           :values #("-1.5" "0.0")))
         '("forward" "forward" "forward"))))

(defparameter *gazebo-go-around-training*
  (list
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "robot_description"
                           :class_name "Robot"
                           :attributes *robot-attributes*
                           :values #("0.0" "0.0" "E" "0.0" "0.0" "E"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "object"
                           :class_name "Object"
                           :attributes *object-attributes*
                           :values #("1.0" "0.0" "E" "1.0" "0.0" "E" 
                                     "0.2" "0.2" "0.2" "0.5" "0" "box" "Gazebo/Blue")))
         '("left" "forward" "right" "forward" "forward" "right" "forward" "left"))
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "robot_description"
                           :class_name "Robot"
                           :attributes *robot-attributes*
                           :values #("0.0" "0.0" "E" "0.0" "0.0" "E"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "object"
                           :class_name "Object"
                           :attributes *object-attributes*
                           :values #("1.0" "0.0" "E" "1.0" "0.0" "E" 
                                     "0.2" "0.2" "0.2" "0.5" "0" "box" "Gazebo/Blue")))
         '("right" "forward" "left" "forward" "forward" "left" "forward" "right"))))




(defparameter *gazebo-deliver-training*
  (list
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "robot_description"
                           :class_name "Robot"
                           :attributes *robot-attributes*
                           :values #("0.0" "0.0" "E" "0.0" "0.0" "E"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "item"
                           :class_name "Item"
                           :attributes *item-attributes*
                           :values #("1.0" "0.0" "1.0" "0.0" "0" "NONE"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "destination"
                           :class_name "Location"
                           :attributes *location-attributes*
                           :values #("2.0" "1.0")))
         '("forward" "forward" "pick_up item" "left" "forward" "forward" "drop"))
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "robot_description"
                           :class_name "Robot"
                           :attributes *robot-attributes*
                           :values #("0.0" "0.0" "E" "0.0" "0.0" "E"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "item"
                           :class_name "Item"
                           :attributes *item-attributes*
                           :values #("1.0" "0.0" "1.0" "0.0" "0" "NONE"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "destination"
                           :class_name "Location"
                           :attributes *location-attributes*
                           :values #("1.5" "0.0")))
         '("forward" "forward" "pick_up item" "forward" "drop"))
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "robot_description"
                           :class_name "Robot"
                           :attributes *robot-attributes*
                           :values #("0.0" "0.0" "E" "0.0" "0.0" "E"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "item"
                           :class_name "Item"
                           :attributes *item-attributes*
                           :values #("1.0" "1.0" "1.0" "1.0" "0" "NONE"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "destination"
                           :class_name "Location"
                           :attributes *location-attributes*
                           :values #("2.0" "0.0")))
         '("left" "forward" "forward" "pick_up item" "right" "right" "forward" "forward" "drop"))
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "robot_description"
                           :class_name "Robot"
                           :attributes *robot-attributes*
                           :values #("0.0" "0.0" "E" "0.0" "0.0" "E"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "item"
                           :class_name "Item"
                           :attributes *item-attributes*
                           :values #("0.0" "0.0" "0.0" "0.0" "1" "robot_description"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "destination"
                           :class_name "Location"
                           :attributes *location-attributes*
                           :values #("2.0" "0.0")))
         '("forward" "forward" "forward" "forward" "drop"))
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "robot_description"
                           :class_name "Robot"
                           :attributes *robot-attributes*
                           :values #("0.0" "0.0" "E" "0.0" "0.0" "E"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "item"
                           :class_name "Item"
                           :attributes *item-attributes*
                           :values #("0.0" "0.0" "0.0" "0.0" "0" "NONE"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "destination"
                           :class_name "Location"
                           :attributes *location-attributes*
                           :values #("2.0" "0.0")))
         '("pick_up item" "forward" "forward" "forward" "forward" "drop"))
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "robot_description"
                           :class_name "Robot"
                           :attributes *robot-attributes*
                           :values #("0.0" "0.0" "E" "0.0" "0.0" "E"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "item"
                           :class_name "Item"
                           :attributes *item-attributes*
                           :values #("1.0" "0.0" "1.0" "0.0" "0" "NONE"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "destination"
                           :class_name "Location"
                           :attributes *location-attributes*
                           :values #("-0.5" "0.0")))
         '("forward" "forward" "pick_up item" "left" "left" "left" "left" "forward" "forward" "forward" "drop"))
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "robot_description"
                           :class_name "Robot"
                           :attributes *robot-attributes*
                           :values #("0.0" "0.0" "E" "0.0" "0.0" "E"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "item"
                           :class_name "Item"
                           :attributes *item-attributes*
                           :values #("0.0" "0.0" "0.0" "0.0" "1" "robot_description"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "destination"
                           :class_name "Location"
                           :attributes *location-attributes*
                           :values #("0.5" "0.0")))
         '("forward" "drop"))
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "robot_description"
                           :class_name "Robot"
                           :attributes *robot-attributes*
                           :values #("0.0" "0.0" "E" "0.0" "0.0" "E"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "item"
                           :class_name "Item"
                           :attributes *item-attributes*
                           :values #("0.0" "0.0" "0.0" "0.0" "1" "robot_description"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "destination"
                           :class_name "Location"
                           :attributes *location-attributes*
                           :values #("-1.0" "0.0")))
         '("left" "left" "left" "left" "forward" "forward" "drop"))
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "robot_description"
                           :class_name "Robot"
                           :attributes *robot-attributes*
                           :values #("0.0" "0.0" "E" "0.0" "0.0" "E"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "item"
                           :class_name "Item"
                           :attributes *item-attributes*
                           :values #("0.0" "0.0" "0.0" "0.0" "1" "robot_description"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "destination"
                           :class_name "Location"
                           :attributes *location-attributes*
                           :values #("0.0" "-0.5")))
         '("right" "right" "forward" "drop"))
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "robot_description"
                           :class_name "Robot"
                           :attributes *robot-attributes*
                           :values #("0.0" "0.0" "N" "0.0" "0.0" "N"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "item"
                           :class_name "Item"
                           :attributes *item-attributes*
                           :values #("-0.5" "0.5" "-0.5" "0.5" "0" "NONE"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "destination"
                           :class_name "Location"
                           :attributes *location-attributes*
                           :values #("0.5" "-0.5")))
         '("left" "forward" "pick_up item" "right" "right" "right" "right" "forward" "forward" "drop"))
))

(defun get-gazebo-training (verb)
  (cond ((string-equal verb "go") *gazebo-go-training*)
        ((string-equal verb "go-through") *gazebo-go-through-training*)
        ((string-equal verb "go-around") *gazebo-go-around-training*)
        ((string-equal verb "deliver") *gazebo-deliver-training*)))