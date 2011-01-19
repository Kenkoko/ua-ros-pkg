(in-package :simsem)

(defparameter *ww2d-go-via-tests*
  (list 
   ;; Training 1: Both ahead, close together
   (vector (make-msg "oomdp_msgs/MDPObjectState"
                     :name "person"
                     :class_name "agent"
                     :attributes *agent-attributes*
                     :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "waypoint"
                     :class_name "obstacle"
                     :attributes *agent-attributes*
                     :values #("54" "50" "0" "0" "0" "0" "circle" "0.3"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "place"
                     :class_name "obstacle"
                     :attributes *agent-attributes*
                     :values #("60" "50" "0" "0" "0" "0" "circle" "0.3")))
   ;; Training 2: Both ahead, further apart
   (vector (make-msg "oomdp_msgs/MDPObjectState"
                     :name "person"
                     :class_name "agent"
                     :attributes *agent-attributes*
                     :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "waypoint"
                     :class_name "obstacle"
                     :attributes *agent-attributes*
                     :values #("54" "50" "0" "0" "0" "0" "circle" "0.3"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "place"
                     :class_name "obstacle"
                     :attributes *agent-attributes*
                     :values #("65" "50" "0" "0" "0" "0" "circle" "0.3")))
  ;; Training 3: ahead then right (close)
   (vector (make-msg "oomdp_msgs/MDPObjectState"
                     :name "person"
                     :class_name "agent"
                     :attributes *agent-attributes*
                     :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "waypoint"
                     :class_name "obstacle"
                     :attributes *agent-attributes*
                     :values #("54" "50" "0" "0" "0" "0" "circle" "0.3"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "place"
                     :class_name "obstacle"
                     :attributes *agent-attributes*
                     :values #("55" "55" "0" "0" "0" "0" "circle" "0.3")))
   ;; Training 4: ahead then left (close)
   (vector (make-msg "oomdp_msgs/MDPObjectState"
                     :name "person"
                     :class_name "agent"
                     :attributes *agent-attributes*
                     :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "waypoint"
                     :class_name "obstacle"
                     :attributes *agent-attributes*
                     :values #("54" "50" "0" "0" "0" "0" "circle" "0.3"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "place"
                     :class_name "obstacle"
                     :attributes *agent-attributes*
                     :values #("55" "45" "0" "0" "0" "0" "circle" "0.3")))
   ;; Training 5: ahead then right (far)
   (vector (make-msg "oomdp_msgs/MDPObjectState"
                     :name "person"
                     :class_name "agent"
                     :attributes *agent-attributes*
                     :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "waypoint"
                     :class_name "obstacle"
                     :attributes *agent-attributes*
                     :values #("55" "50" "0" "0" "0" "0" "circle" "0.3"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "place"
                     :class_name "obstacle"
                     :attributes *agent-attributes*
                     :values #("60" "60" "0" "0" "0" "0" "circle" "0.3")))
   ;; Training 6: ahead then left (far)
   (vector (make-msg "oomdp_msgs/MDPObjectState"
                     :name "person"
                     :class_name "agent"
                     :attributes *agent-attributes*
                     :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "waypoint"
                     :class_name "obstacle"
                     :attributes *agent-attributes*
                     :values #("55" "50" "0" "0" "0" "0" "circle" "0.3"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "place"
                     :class_name "obstacle"
                     :attributes *agent-attributes*
                     :values #("62" "41" "0" "0" "0" "0" "circle" "0.3")))
   ;; Training 7: behind then in front 
   (vector (make-msg "oomdp_msgs/MDPObjectState"
                     :name "person"
                     :class_name "agent"
                     :attributes *agent-attributes*
                     :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "waypoint"
                     :class_name "obstacle"
                     :attributes *agent-attributes*
                     :values #("45" "50" "0" "0" "0" "0" "circle" "0.3"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "place"
                     :class_name "obstacle"
                     :attributes *agent-attributes*
                     :values #("54" "50" "0" "0" "0" "0" "circle" "0.3")))         
   ;; Training 8: both right ahead
   (vector (make-msg "oomdp_msgs/MDPObjectState"
                     :name "person"
                     :class_name "agent"
                     :attributes *agent-attributes*
                     :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "waypoint"
                     :class_name "obstacle"
                     :attributes *agent-attributes*
                     :values #("54" "54" "0" "0" "0" "0" "circle" "0.3"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "place"
                     :class_name "obstacle"
                     :attributes *agent-attributes*
                     :values #("60" "56" "0" "0" "0" "0" "circle" "0.3")))
   ;; Training 9: both left ahead
   (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "person"
                           :class_name "agent"
                           :attributes *agent-attributes*
                           :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "waypoint"
                           :class_name "obstacle"
                           :attributes *agent-attributes*
                           :values #("55" "47" "0" "0" "0" "0" "circle" "0.3"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "place"
                           :class_name "obstacle"
                           :attributes *agent-attributes*
                           :values #("62" "43" "0" "0" "0" "0" "circle" "0.3")))
))




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
                          :values #("42" "42" "0" "0" "0" "0" "circle" "0.5")))
        ;; This was removed...
        (vector (make-msg "oomdp_msgs/MDPObjectState"
                          :name "person"
                          :class_name "agent"
                          :attributes *agent-attributes*
                          :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
                (make-msg "oomdp_msgs/MDPObjectState"
                          :name "place"
                          :class_name "obstacle"
                          :attributes *agent-attributes*
                          :values #("40" "50" "0" "0" "0" "0" "circle" "0.5")))
        (vector (make-msg "oomdp_msgs/MDPObjectState"
                          :name "person"
                          :class_name "agent"
                          :attributes *agent-attributes*
                          :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
                (make-msg "oomdp_msgs/MDPObjectState"
                          :name "place"
                          :class_name "obstacle"
                          :attributes *agent-attributes*
                          :values #("57" "57" "0" "0" "0" "0" "circle" "0.5")))
        (vector (make-msg "oomdp_msgs/MDPObjectState"
                          :name "person"
                          :class_name "agent"
                          :attributes *agent-attributes*
                          :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
                (make-msg "oomdp_msgs/MDPObjectState"
                          :name "place"
                          :class_name "obstacle"
                          :attributes *agent-attributes*
                          :values #("57" "50" "0" "0" "0" "0" "circle" "0.5")))
        ;; This was removed...
        (vector (make-msg "oomdp_msgs/MDPObjectState"
                          :name "person"
                          :class_name "agent"
                          :attributes *agent-attributes*
                          :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
                (make-msg "oomdp_msgs/MDPObjectState"
                          :name "place"
                          :class_name "obstacle"
                          :attributes *agent-attributes*
                          :values #("60" "52" "0" "0" "0" "0" "circle" "0.5")))
        (vector (make-msg "oomdp_msgs/MDPObjectState"
                          :name "person"
                          :class_name "agent"
                          :attributes *agent-attributes*
                          :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
                (make-msg "oomdp_msgs/MDPObjectState"
                          :name "place"
                          :class_name "obstacle"
                          :attributes *agent-attributes*
                          :values #("56" "44" "0" "0" "0" "0" "circle" "0.5")))
        (vector (make-msg "oomdp_msgs/MDPObjectState"
                          :name "person"
                          :class_name "agent"
                          :attributes *agent-attributes*
                          :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
                (make-msg "oomdp_msgs/MDPObjectState"
                          :name "place"
                          :class_name "obstacle"
                          :attributes *agent-attributes*
                          :values #("55" "51" "0" "0" "0" "0" "circle" "0.5")))
        (vector (make-msg "oomdp_msgs/MDPObjectState"
                          :name "person"
                          :class_name "agent"
                          :attributes *agent-attributes*
                          :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
                (make-msg "oomdp_msgs/MDPObjectState"
                          :name "place"
                          :class_name "obstacle"
                          :attributes *agent-attributes*
                          :values #("56" "47" "0" "0" "0" "0" "circle" "0.5")))))

(defun get-ww2d-tests (verb)
  (cond ((string-equal verb "go") *ww2d-go-tests*)
        ((string-equal verb "go-via") *ww2d-go-via-tests*)))
