(in-package :simsem)

(defparameter *agent-attributes* #("x" "y" "heading" 
                                   "vx" "vy" "vtheta"
                                   "shape-type" "radius"))

(defparameter *ww2d-intercept-training*
  (list 
   ;; Training 1: Diagonal from below
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "person"
                           :class_name "agent"
                           :attributes *agent-attributes*
                           :values #("50" "55" "-3" "0" "0" "0" "circle" "0.25"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "enemy"
                           :class_name "agent"
                           :attributes *agent-attributes*
                           :values #("40" "50" "0" "0" "0" "0" "circle" "0.25"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "place"
                           :class_name "obstacle"
                           :attributes *agent-attributes*
                           :values #("70" "50" "0" "0" "0" "0" "circle" "0.5")))
         '("person 1000" "person 1000" "person 1000" "person 0000"
           "person 0000" "person 0000" "person 0000"))
   ;; Training 2: Training 1 reversed
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "person"
                           :class_name "agent"
                           :attributes *agent-attributes*
                           :values #("50" "45" "3" "0" "0" "0" "circle" "0.25"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "enemy"
                           :class_name "agent"
                           :attributes *agent-attributes*
                           :values #("40" "50" "0" "0" "0" "0" "circle" "0.25"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "place"
                           :class_name "obstacle"
                           :attributes *agent-attributes*
                           :values #("70" "50" "0" "0" "0" "0" "circle" "0.5")))
         '("person 1000" "person 1000" "person 1000" "person 0000"
           "person 0000" "person 0000" "person 0000"))
   ;; Training 3: Directly in front
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "person"
                           :class_name "agent"
                           :attributes *agent-attributes*
                           :values #("50" "50" "-4" "0" "0" "0" "circle" "0.25"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "enemy"
                           :class_name "agent"
                           :attributes *agent-attributes*
                           :values #("40" "50" "0" "0" "0" "0" "circle" "0.25"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "place"
                           :class_name "obstacle"
                           :attributes *agent-attributes*
                           :values #("70" "50" "0" "0" "0" "0" "circle" "0.5")))
         '("person 1000" "person 1000" "person 1000" "person 1000"
           "person 1000" "person 1000"))
   ;; Training 4: parallel (curl pattern)
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "person"
                           :class_name "agent"
                           :attributes *agent-attributes*
                           :values #("40" "55" "0" "0" "0" "0" "circle" "0.25"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "enemy"
                           :class_name "agent"
                           :attributes *agent-attributes*
                           :values #("40" "50" "0" "0" "0" "0" "circle" "0.25"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "place"
                           :class_name "obstacle"
                           :attributes *agent-attributes*
                           :values #("70" "50" "0" "0" "0" "0" "circle" "0.5")))
         '("person 1000" "person 1000" "person 1100" "person 1100"
           "person 1100" "person 0000" "person 0100" "person 0100"
           "person 0100" "person 0000" "person 0000" "person 0000"
           "person 0000" "person 0000" "person 0000" "person 0000"
           "person 0000" "person 0000" "person 0000" "person 0000"
           "person 0000"))
   ;; Training 5: Directly behind
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "person"
                           :class_name "agent"
                           :attributes *agent-attributes*
                           :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "enemy"
                           :class_name "agent"
                           :attributes *agent-attributes*
                           :values #("40" "50" "0" "0" "0" "0" "circle" "0.25"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "place"
                           :class_name "obstacle"
                           :attributes *agent-attributes*
                           :values #("70" "50" "0" "0" "0" "0" "circle" "0.5")))
         '("person 0001" "person 0001" "person 0001" "person 0001"
           "person 0001" "person 0001"))
   ;; Training 6: Catch from behind
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "person"
                           :class_name "agent"
                           :attributes *agent-attributes*
                           :values #("40" "50" "0" "0" "0" "0" "circle" "0.25"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "enemy"
                           :class_name "agent"
                           :attributes *agent-attributes*
                           :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "place"
                           :class_name "obstacle"
                           :attributes *agent-attributes*
                           :values #("70" "50" "0" "0" "0" "0" "circle" "0.5")))
         '("person 1000" "person 1000" "person 1000" "person 1000"
           "person 1000" "person 1000" "person 1000" "person 1000" "person 1000"))
   ;; Training 7: near the goal
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "person"
                           :class_name "agent"
                           :attributes *agent-attributes*
                           :values #("70" "60" "-3" "0" "0" "0" "circle" "0.25"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "enemy"
                           :class_name "agent"
                           :attributes *agent-attributes*
                           :values #("40" "50" "0" "0" "0" "0" "circle" "0.25"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "place"
                           :class_name "obstacle"
                           :attributes *agent-attributes*
                           :values #("70" "50" "0" "0" "0" "0" "circle" "0.5")))
         '("person 1000" "person 1000" "person 1000" "person 0100"
           "person 0100" "person 1000" "person 1000" "person 1000"
           "person 1000" "person 1100" "person 1000" "person 1000"
           "person 1000" "person 1000"))
   ;; Training 8: Behind at an angle
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "person"
                           :class_name "agent"
                           :attributes *agent-attributes*
                           :values #("40" "60" "-1" "0" "0" "0" "circle" "0.25"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "enemy"
                           :class_name "agent"
                           :attributes *agent-attributes*
                           :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "place"
                           :class_name "obstacle"
                           :attributes *agent-attributes*
                           :values #("70" "50" "0" "0" "0" "0" "circle" "0.5")))
         '("person 1010" "person 1000" "person 1000" "person 1000"
           "person 1000" "person 1000" "person 1000" "person 1000"
           "person 1000" "person 1000"))
))



(defparameter *ww2d-go-around-training*
  (list 
   ;; Training 1: 
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
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
         '("person 1000" "person 1000" "person 1000" "person 1000" 
           "person 1000" "person 1000"))
))



(defparameter *ww2d-go-via-training*
  (list 
   ;; Training 1: Both ahead, close together
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "person"
                           :class_name "agent"
                           :attributes *agent-attributes*
                           :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "waypoint"
                           :class_name "obstacle"
                           :attributes *agent-attributes*
                           :values #("60" "50" "0" "0" "0" "0" "circle" "0.5"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "place"
                           :class_name "obstacle"
                           :attributes *agent-attributes*
                           :values #("70" "50" "0" "0" "0" "0" "circle" "0.5")))
         '("person 1000" "person 1000" "person 1000" "person 1000"
           "person 1000" "person 1000" "person 1000" "person 1000"
           "person 1000" "person 1000" 
           ))
   ;; Training 2: Both ahead, further away
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "person"
                           :class_name "agent"
                           :attributes *agent-attributes*
                           :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "waypoint"
                           :class_name "obstacle"
                           :attributes *agent-attributes*
                           :values #("60" "50" "0" "0" "0" "0" "circle" "0.5"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "place"
                           :class_name "obstacle"
                           :attributes *agent-attributes*
                           :values #("73" "50" "0" "0" "0" "0" "circle" "0.5")))
         '("person 1000" "person 1000" "person 1000" "person 1000" 
           "person 1000" "person 1000" "person 1000" "person 1000" 
           "person 1000" "person 1000" "person 1000"))
   ;; Training 3: ahead then right (close)
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "person"
                           :class_name "agent"
                           :attributes *agent-attributes*
                           :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "waypoint"
                           :class_name "obstacle"
                           :attributes *agent-attributes*
                           :values #("54" "50" "0" "0" "0" "0" "circle" "0.5"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "place"
                           :class_name "obstacle"
                           :attributes *agent-attributes*
                           :values #("60" "60" "0" "0" "0" "0" "circle" "0.5")))
         '("person 1000"  "person 0010" "person 0010" "person 1000"
           "person 0010" "person 0010" "person 1000" "person 1000" 
           "person 1000" "person 1000" "person 1000" "person 1000"))
   ;; Training 4: ahead then left (close)
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "person"
                           :class_name "agent"
                           :attributes *agent-attributes*
                           :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "waypoint"
                           :class_name "obstacle"
                           :attributes *agent-attributes*
                           :values #("54" "50" "0" "0" "0" "0" "circle" "0.5"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "place"
                           :class_name "obstacle"
                           :attributes *agent-attributes*
                           :values #("55" "45" "0" "0" "0" "0" "circle" "0.5")))
         '("person 1000"  "person 0100" "person 0100" "person 1000"
           "person 0100" "person 0100" "person 1000"  "person 1000"
           "person 1000"))
   ;; Training 5: ahead then right (far)
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "person"
                           :class_name "agent"
                           :attributes *agent-attributes*
                           :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "waypoint"
                           :class_name "obstacle"
                           :attributes *agent-attributes*
                           :values #("55" "50" "0" "0" "0" "0" "circle" "0.5"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "place"
                           :class_name "obstacle"
                           :attributes *agent-attributes*
                           :values #("60" "60" "0" "0" "0" "0" "circle" "0.5")))
         '("person 1000" "person 0010" "person 1000" "person 1000" 
           "person 0010" "person 0010" "person 0010" "person 1000" 
           "person 1000" "person 1000" "person 1000" "person 1000" 
           "person 1000"))
   ;; Training 6: ahead then left (far)
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "person"
                           :class_name "agent"
                           :attributes *agent-attributes*
                           :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "waypoint"
                           :class_name "obstacle"
                           :attributes *agent-attributes*
                           :values #("55" "50" "0" "0" "0" "0" "circle" "0.5"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "place"
                           :class_name "obstacle"
                           :attributes *agent-attributes*
                           :values #("62" "41" "0" "0" "0" "0" "circle" "0.5")))
         '("person 1000" "person 0100" "person 1000" "person 0100" "person 1000"
           "person 0100" "person 0100" "person 1000" "person 1000" "person 1000" 
           "person 1000" "person 1000" "person 1000"))
   ;; Training 7: behind then in front 
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "person"
                           :class_name "agent"
                           :attributes *agent-attributes*
                           :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "waypoint"
                           :class_name "obstacle"
                           :attributes *agent-attributes*
                           :values #("45" "50" "0" "0" "0" "0" "circle" "0.5"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "place"
                           :class_name "obstacle"
                           :attributes *agent-attributes*
                           :values #("54" "50" "0" "0" "0" "0" "circle" "0.5")))
         '("person 0100" "person 0100" "person 0100" "person 0100" 
           "person 0100" "person 1000" "person 0100" "person 1000"
           "person 0100" "person 0100" "person 0100" "person 0100"
           "person 0100" "person 1000" "person 0100" "person 1000"
           "person 1000" "person 1000" "person 1000" "person 1000"
           "person 1000"))
   ;; Training 8: both right ahead
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "person"
                           :class_name "agent"
                           :attributes *agent-attributes*
                           :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "waypoint"
                           :class_name "obstacle"
                           :attributes *agent-attributes*
                           :values #("54" "54" "0" "0" "0" "0" "circle" "0.5"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "place"
                           :class_name "obstacle"
                           :attributes *agent-attributes*
                           :values #("60" "56" "0" "0" "0" "0" "circle" "0.5")))
         '("person 0010" "person 0010" "person 1000" "person 1000" 
           "person 0010" "person 1000" "person 1000" "person 1000" 
           "person 1000" "person 0100"
           ))
   ;; Training 9: both left ahead
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "person"
                           :class_name "agent"
                           :attributes *agent-attributes*
                           :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "waypoint"
                           :class_name "obstacle"
                           :attributes *agent-attributes*
                           :values #("55" "47" "0" "0" "0" "0" "circle" "0.5"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "place"
                           :class_name "obstacle"
                           :attributes *agent-attributes*
                           :values #("62" "43" "0" "0" "0" "0" "circle" "0.5")))
         '("person 0100" "person 0100" "person 1000" "person 1000" 
           "person 0100" "person 1000" "person 1000" "person 1000" 
           "person 1000" "person 0010" "person 0010" "person 1000"
           ))
   ;; Training 10: bam bam
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "person"
                           :class_name "agent"
                           :attributes *agent-attributes*
                           :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "waypoint"
                           :class_name "obstacle"
                           :attributes *agent-attributes*
                           :values #("55" "50" "0" "0" "0" "0" "circle" "0.5"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "place"
                           :class_name "obstacle"
                           :attributes *agent-attributes*
                           :values #("60" "50" "0" "0" "0" "0" "circle" "0.5")))
         '("person 1000" "person 1000" "person 1000" "person 1000"
           "person 1000" "person 1000"))
))






(defparameter *ww2d-go-training*
  (list 
   ;; Training 1: 5 units behind
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "person"
                           :class_name "agent"
                           :attributes *agent-attributes*
                           :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "place"
                           :class_name "obstacle"
                           :attributes *agent-attributes*
                           :values #("45" "50" "0" "0" "0" "0" "circle" "0.5")))
         '("person 0100" "person 0100" "person 0100" "person 0100" "person 0100"
                        "person 1000" "person 0100" "person 1000"))
   ;; Training 2: 5 units in front
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "person"
                           :class_name "agent"
                           :attributes *agent-attributes*
                           :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "place"
                           :class_name "obstacle"
                           :attributes *agent-attributes*
                           :values #("55" "50" "0" "0" "0" "0" "circle" "0.5")))
         '("person 1000" "person 1000" "person 1000"))
   ;; Training 3: 5 units to the right
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "person"
                           :class_name "agent"
                           :attributes *agent-attributes*
                           :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "place"
                           :class_name "obstacle"
                           :attributes *agent-attributes*
                           :values #("50" "55" "0" "0" "0" "0" "circle" "0.5")))
         '("person 0010" "person 0010" "person 0010" "person 1000" "person 1000" "person 1000"))
   ;; Training 4: 5 units to the left
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "person"
                           :class_name "agent"
                           :attributes *agent-attributes*
                           :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "place"
                           :class_name "obstacle"
                           :attributes *agent-attributes*
                           :values #("50" "45" "0" "0" "0" "0" "circle" "0.5")))
         '("person 0100" "person 0100" "person 0100" "person 1000" "person 1000" "person 1000"))
   ;; Training 5: 5 units to the left, 5 forward
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "person"
                           :class_name "agent"
                           :attributes *agent-attributes*
                           :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "place"
                           :class_name "obstacle"
                           :attributes *agent-attributes*
                           :values #("55" "45" "0" "0" "0" "0" "circle" "0.5")))
         '("person 0100" "person 0100" "person 1000" "person 1000" 
           "person 1000" "person 1000" "person 1000"))
   ;; Training 6: 5 units to the right, 5 units forward
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "person"
                           :class_name "agent"
                           :attributes *agent-attributes*
                           :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "place"
                           :class_name "obstacle"
                           :attributes *agent-attributes*
                           :values #("55" "55" "0" "0" "0" "0" "circle" "0.5")))
         '("person 0010" "person 0010" "person 1000" "person 1000" 
           "person 1000" "person 1000" "person 1000"))
   ;; Training 7: 5 units to the right, 5 units back
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "person"
                           :class_name "agent"
                           :attributes *agent-attributes*
                           :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "place"
                           :class_name "obstacle"
                           :attributes *agent-attributes*
                           :values #("45" "55" "0" "0" "0" "0" "circle" "0.5")))
         '("person 0010" "person 0010" "person 0010" "person 0010" 
           "person 1000" "person 0010" "person 1000" "person 1000" "person 1000"))
   ;; Training 8: 5 units to the left, 5 units back
   (list (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "person"
                           :class_name "agent"
                           :attributes *agent-attributes*
                           :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "place"
                           :class_name "obstacle"
                           :attributes *agent-attributes*
                           :values #("45" "45" "0" "0" "0" "0" "circle" "0.5")))
         '("person 0100" "person 0100" "person 0100" "person 0100" 
           "person 1000" "person 0100" "person 1000" "person 1000" "person 1000"))
))

(defun get-ww2d-training (verb)
  (cond ((string-equal verb "go") *ww2d-go-training*)
        ((string-equal verb "go-via") *ww2d-go-via-training*)
        ((string-equal verb "intercept") *ww2d-intercept-training*)))