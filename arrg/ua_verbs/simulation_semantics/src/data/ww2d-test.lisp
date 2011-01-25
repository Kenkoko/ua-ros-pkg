(in-package :simsem)

(defparameter *ww2d-intercept-tests*
  (list 
   ;; Test 1: Below, facing enemy
   (vector (make-msg "oomdp_msgs/MDPObjectState"
                     :name "person"
                     :class_name "agent"
                     :attributes *agent-attributes*
                     :values #("55" "55" "-3" "0" "0" "0" "circle" "0.25"))
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
   ;; Test 2: Above, facing enemy
   (vector (make-msg "oomdp_msgs/MDPObjectState"
                     :name "person"
                     :class_name "agent"
                     :attributes *agent-attributes*
                     :values #("55" "45" "3" "0" "0" "0" "circle" "0.25"))
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
   ;; Test 3: In line, facing the enemy (easy!) 
   (vector (make-msg "oomdp_msgs/MDPObjectState"
                     :name "person"
                     :class_name "agent"
                     :attributes *agent-attributes*
                     :values #("55" "50" "4" "0" "0" "0" "circle" "0.25"))
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
   ;; Test 4: Below, facing due north 
   (vector (make-msg "oomdp_msgs/MDPObjectState"
                     :name "person"
                     :class_name "agent"
                     :attributes *agent-attributes*
                     :values #("55" "55" "-2" "0" "0" "0" "circle" "0.25"))
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
   ;; Test 5: Above, facing due south 
   (vector (make-msg "oomdp_msgs/MDPObjectState"
                     :name "person"
                     :class_name "agent"
                     :attributes *agent-attributes*
                     :values #("55" "45" "2" "0" "0" "0" "circle" "0.25"))
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
   ;; Test 6: Enemy behind
   (vector (make-msg "oomdp_msgs/MDPObjectState"
                     :name "person"
                     :class_name "agent"
                     :attributes *agent-attributes*
                     :values #("60" "50" "1" "0" "0" "0" "circle" "0.25"))
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
   ;; Test 7: Behind
   (vector (make-msg "oomdp_msgs/MDPObjectState"
                     :name "person"
                     :class_name "agent"
                     :attributes *agent-attributes*
                     :values #("47" "50" "0" "0" "0" "0" "circle" "0.25"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "enemy"
                     :class_name "agent"
                     :attributes *agent-attributes*
                     :values #("53" "50" "0" "0" "0" "0" "circle" "0.25"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "place"
                     :class_name "obstacle"
                     :attributes *agent-attributes*
                     :values #("70" "50" "0" "0" "0" "0" "circle" "0.5")))
    ;; Test 8: near the goal
   (vector (make-msg "oomdp_msgs/MDPObjectState"
                     :name "person"
                     :class_name "agent"
                     :attributes *agent-attributes*
                     :values #("65" "40" "3" "0" "0" "0" "circle" "0.25"))
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
))

(defparameter *ww2d-go-via-tests*
  (list 
   ;; Test 1: Both ahead 1
   (vector (make-msg "oomdp_msgs/MDPObjectState"
                     :name "person"
                     :class_name "agent"
                     :attributes *agent-attributes*
                     :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "waypoint"
                     :class_name "obstacle"
                     :attributes *agent-attributes*
                     :values #("57" "50" "0" "0" "0" "0" "circle" "0.5"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "place"
                     :class_name "obstacle"
                     :attributes *agent-attributes*
                     :values #("68" "50" "0" "0" "0" "0" "circle" "0.5")))
   ;; Test 2: Both ahead 2
   (vector (make-msg "oomdp_msgs/MDPObjectState"
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
   ;; Test 3: Both ahead 3
   (vector (make-msg "oomdp_msgs/MDPObjectState"
                     :name "person"
                     :class_name "agent"
                     :attributes *agent-attributes*
                     :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "waypoint"
                     :class_name "obstacle"
                     :attributes *agent-attributes*
                     :values #("57" "50" "0" "0" "0" "0" "circle" "0.5"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "place"
                     :class_name "obstacle"
                     :attributes *agent-attributes*
                     :values #("62" "50" "0" "0" "0" "0" "circle" "0.5")))
   ;; Test 4: Ahead then right
   (vector (make-msg "oomdp_msgs/MDPObjectState"
                     :name "person"
                     :class_name "agent"
                     :attributes *agent-attributes*
                     :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "waypoint"
                     :class_name "obstacle"
                     :attributes *agent-attributes*
                     :values #("57" "50" "0" "0" "0" "0" "circle" "0.5"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "place"
                     :class_name "obstacle"
                     :attributes *agent-attributes*
                     :values #("63" "55" "0" "0" "0" "0" "circle" "0.5")))
   ;; Test 5: Ahead then left
   (vector (make-msg "oomdp_msgs/MDPObjectState"
                     :name "person"
                     :class_name "agent"
                     :attributes *agent-attributes*
                     :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "waypoint"
                     :class_name "obstacle"
                     :attributes *agent-attributes*
                     :values #("57" "50" "0" "0" "0" "0" "circle" "0.5"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "place"
                     :class_name "obstacle"
                     :attributes *agent-attributes*
                     :values #("64" "43" "0" "0" "0" "0" "circle" "0.5")))
   ;; Test 6: right more right
   (vector (make-msg "oomdp_msgs/MDPObjectState"
                     :name "person"
                     :class_name "agent"
                     :attributes *agent-attributes*
                     :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "waypoint"
                     :class_name "obstacle"
                     :attributes *agent-attributes*
                     :values #("56" "56" "0" "0" "0" "0" "circle" "0.5"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "place"
                     :class_name "obstacle"
                     :attributes *agent-attributes*
                     :values #("66" "59" "0" "0" "0" "0" "circle" "0.5")))
   ;; Test 7: left more left
   (vector (make-msg "oomdp_msgs/MDPObjectState"
                     :name "person"
                     :class_name "agent"
                     :attributes *agent-attributes*
                     :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "waypoint"
                     :class_name "obstacle"
                     :attributes *agent-attributes*
                     :values #("57" "45" "0" "0" "0" "0" "circle" "0.5"))
           (make-msg "oomdp_msgs/MDPObjectState"
                     :name "place"
                     :class_name "obstacle"
                     :attributes *agent-attributes*
                     :values #("67" "40" "0" "0" "0" "0" "circle" "0.5")))
   ;; Test 8: right right
   (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "person"
                           :class_name "agent"
                           :attributes *agent-attributes*
                           :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "waypoint"
                           :class_name "obstacle"
                           :attributes *agent-attributes*
                           :values #("50" "57" "0" "0" "0" "0" "circle" "0.5"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "place"
                           :class_name "obstacle"
                           :attributes *agent-attributes*
                           :values #("50" "65" "0" "0" "0" "0" "circle" "0.5")))
   ;; Test 9: left left (tight)
   (vector (make-msg "oomdp_msgs/MDPObjectState"
                           :name "person"
                           :class_name "agent"
                           :attributes *agent-attributes*
                           :values #("50" "50" "0" "0" "0" "0" "circle" "0.25"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "waypoint"
                           :class_name "obstacle"
                           :attributes *agent-attributes*
                           :values #("50" "42" "0" "0" "0" "0" "circle" "0.5"))
                 (make-msg "oomdp_msgs/MDPObjectState"
                           :name "place"
                           :class_name "obstacle"
                           :attributes *agent-attributes*
                           :values #("50" "37" "0" "0" "0" "0" "circle" "0.5")))
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
        ((string-equal verb "go-via") *ww2d-go-via-tests*)
        ((string-equal verb "intercept") *ww2d-intercept-tests*)))
