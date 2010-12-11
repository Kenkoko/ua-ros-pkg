(in-package :simsem)

(defparameter *ww2d-go-tests*
  (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                          :name "person"
                          :class_name "agent"
                          :attributes *agent-attributes*
                          :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
                (make-msg "oomdp_msgs/MDPObjectState"
                          :name "place"
                          :class_name "obstacle"
                          :attributes *agent-attributes*
                          :values #("75" "50" "0" "0" "0" "0" "circle" "0.5")))))

(defun get-ww2d-tests (verb)
  (cond ((string-equal verb "go") *ww2d-go-tests*)))
        