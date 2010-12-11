(in-package :simsem)

(defparameter *agent-attributes* #("x" "y" "heading" 
                                   "vx" "vy" "vtheta"
                                   "shape-type" "radius"))

(defparameter *ww2d-go-training*
  (list 
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "person"
                           :class_name "agent"
                           :attributes *agent-attributes*
                           :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "place"
                           :class_name "obstacle"
                           :attributes *agent-attributes*
                           :values #("75" "50" "0" "0" "0" "0" "circle" "0.5")))
         (loop for i below 9 collect "person 1000"))
   ))

(defun get-ww2d-training (verb)
  (cond ((string-equal verb "go") *ww2d-go-training*)))