(in-package :simsem)

(defparameter *robot-attributes* #("x" "y" "orientation" "last_x" "last_y" "last_orientation"))
(defparameter *location-attributes* #("x" "y"))
(defparameter *object-attributes* #("x" "y" "orientation" "last_x" "last_y" "last_orientation"
                                    "size_x" "size_y" "size_z" "mass" "static" "shape" "color"))
(defparameter *agent-attributes* #("x" "y" "heading" 
                                   "vx" "vy" "vtheta"
                                   "shape-type" "radius"))

;;=============================================================================
;; Training

;; TODO: Need a way to distinguish domains
(defun make-training (verb)
  (cond ((string-equal verb "go")
         (make-go-training))
        ((string-equal verb "go-around")
         (make-go-around-training))
        ((string-equal verb "go-through")
         (make-go-through-training))
        ((string-equal verb "approach")
         (make-approach-training))))

(defun make-approach-training ()
  (list 
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "mover"
                           :class_name "agent"
                           :attributes *agent-attributes*
                           :values #("5" "5" "0" "0" "0" "0" "circle" "0.25"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "stander"
                           :class_name "agent"
                           :attributes *agent-attributes*
                           :values #("10" "5" "0" "0" "0" "0" "circle" "0.25")))
         (loop for i below 10 collect "mover 1000;stander 0000"))
   ))

(defun make-go-training ()
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
   ))

(defun make-go-through-training ()
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

(defun make-go-around-training ()
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

;;=============================================================================
;; Testing

(defun make-tests (verb)
  (cond ((string-equal verb "go")
         (make-go-tests))
        ((string-equal verb "go-around")
         (make-go-around-tests))
        ((string-equal verb "go-through")
         (make-go-through-tests))
        ((string-equal verb "approach")
         (make-approach-tests))))

(defun make-approach-tests ()
  (list
   (vector (make-msg "oomdp_msgs/MDPObjectState"
                     :name "mover"
                     :class_name "agent"
                     :attributes *agent-attributes*
                     :values #("5" "5" "0" "0" "0" "0" "circle" "0.25"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "stander"
                     :class_name "agent"
                     :attributes *agent-attributes*
                     :values #("10" "5" "0" "0" "0" "0" "circle" "0.25")))))

(defun make-go-tests ()
  "Returns a list of vectors of MDPObjectStates, the start states for each test episode."
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

(defun make-go-through-tests ()
  "Returns a list of vectors of MDPObjectStates, the start states for each test episode."
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

(defun make-go-around-tests ()
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