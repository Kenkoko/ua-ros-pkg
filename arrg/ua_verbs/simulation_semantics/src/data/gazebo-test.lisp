(in-package :simsem)

(defparameter *gazebo-go-tests* 
  (list
   (vector (make-msg "oomdp_msgs/MDPObjectState"
                     :name "robot_description"
                     :class_name "Robot"
                     :attributes *robot-attributes*
                     :values #("0.0" "0.0" "W" "0.0" "0.0" "W"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "goal"
                     :class_name "Location"
                     :attributes *location-attributes*
                     :values #("-2.0" "0.0")))
   (vector (make-msg "oomdp_msgs/MDPObjectState"
                     :name "robot_description"
                     :class_name "Robot"
                     :attributes *robot-attributes*
                     :values #("0.0" "0.0" "N" "0.0" "0.0" "N"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "goal"
                     :class_name "Location"
                     :attributes *location-attributes*
                     :values #("1.0" "1.0")))
   (vector (make-msg "oomdp_msgs/MDPObjectState"
                     :name "robot_description"
                     :class_name "Robot"
                     :attributes *robot-attributes*
                     :values #("0.0" "0.0" "S" "0.0" "0.0" "S"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "goal"
                     :class_name "Location"
                     :attributes *location-attributes*
                     :values #("0.0" "0.5")))
   (vector (make-msg "oomdp_msgs/MDPObjectState"
                     :name "robot_description"
                     :class_name "Robot"
                     :attributes *robot-attributes*
                     :values #("0.0" "0.0" "E" "0.0" "0.0" "E"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "goal"
                     :class_name "Location"
                     :attributes *location-attributes*
                     :values #("0.0" "0.5")))
   ))

(defparameter *gazebo-go-through-tests* 
  (list
   (vector (make-msg "oomdp_msgs/MDPObjectState"
                     :name "robot_description"
                     :class_name "Robot"
                     :attributes *robot-attributes*
                     :values #("0.0" "0.0" "E" "0.0" "0.0" "E"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "waypoint"
                     :class_name "Location"
                     :attributes *location-attributes*
                     :values #("1.0" "0.0"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "goal"
                     :class_name "Location"
                     :attributes *location-attributes*
                     :values #("1.5" "0.0")))
   (vector (make-msg "oomdp_msgs/MDPObjectState"
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
                     :values #("1.0" "1.0")))
   (vector (make-msg "oomdp_msgs/MDPObjectState"
                     :name "robot_description"
                     :class_name "Robot"
                     :attributes *robot-attributes*
                     :values #("0.0" "0.0" "SW" "0.0" "0.0" "SW"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "waypoint"
                     :class_name "Location"
                     :attributes *location-attributes*
                     :values #("-0.5" "-0.5"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "goal"
                     :class_name "Location"
                     :attributes *location-attributes*
                     :values #("0.5" "0.5")))
   (vector (make-msg "oomdp_msgs/MDPObjectState"
                     :name "robot_description"
                     :class_name "Robot"
                     :attributes *robot-attributes*
                     :values #("0.0" "0.0" "W" "0.0" "0.0" "W"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "waypoint"
                     :class_name "Location"
                     :attributes *location-attributes*
                     :values #("-0.5" "0.0"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "goal"
                     :class_name "Location"
                     :attributes *location-attributes*
                     :values #("-1.0" "0.0")))
   ))

(defparameter *gazebo-go-around-tests* 
  (list
   (vector (make-msg "oomdp_msgs/MDPObjectState"
                     :name "robot_description"
                     :class_name "Robot"
                     :attributes *robot-attributes*
                     :values #("5.0" "1.0" "S" "5.0" "1.0" "S"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "object"
                     :class_name "Object"
                     :attributes *object-attributes*
                     :values #("5.0" "0.0" "S" "5.0" "0.0" "S" 
                               "0.2" "0.2" "0.2" "0.5" "0" "box" "Gazebo/Blue")))
   (vector (make-msg "oomdp_msgs/MDPObjectState"
                     :name "robot_description"
                     :class_name "Robot"
                     :attributes *robot-attributes*
                     :values #("-5.0" "1.0" "S" "-5.0" "1.0" "S"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "object"
                     :class_name "Object"
                     :attributes *object-attributes*
                     :values #("-5.0" "0.0" "S" "-5.0" "0.0" "S" 
                               "0.2" "0.2" "0.2" "0.5" "0" "box" "Gazebo/Blue")))
   ))

(defparameter *gazebo-deliver-tests*
  (list
   ;; Test 1: Not holding, left
   (vector (make-msg "oomdp_msgs/MDPObjectState"
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
                     :values #("2.0" "0.0")))
   ;; Test 2: Not holding, behind
   (vector (make-msg "oomdp_msgs/MDPObjectState"
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
                     :values #("-1.0" "0.0")))
   ;; Test 3: Holding, dest in front
   (vector (make-msg "oomdp_msgs/MDPObjectState"
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
   ;; Test 4: Holding, dest behind
   (vector (make-msg "oomdp_msgs/MDPObjectState"
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
                     :values #("-2.0" "0.0")))
   ;; Test 5: Not holding, at object
   (vector (make-msg "oomdp_msgs/MDPObjectState"
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
                     :values #("1.0" "0.0")))
   ;; Test 6: Not holding, minimal spacing
   (vector (make-msg "oomdp_msgs/MDPObjectState"
                     :name "robot_description"
                     :class_name "Robot"
                     :attributes *robot-attributes*
                     :values #("0.0" "0.0" "E" "0.0" "0.0" "E"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "item"
                     :class_name "Item"
                     :attributes *item-attributes*
                     :values #("0.5" "0.0" "0.0" "0.0" "0" "NONE"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "destination"
                     :class_name "Location"
                     :attributes *location-attributes*
                     :values #("1.0" "0.0")))
   ))   

(defun get-gazebo-tests (verb)
  (cond ((string-equal verb "go") *gazebo-go-tests*)
        ((string-equal verb "go-through") *gazebo-go-through-tests*)
        ((string-equal verb "go-around") *gazebo-go-around-tests*)
        ((string-equal verb "deliver") *gazebo-deliver-tests*)))
