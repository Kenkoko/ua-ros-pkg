(in-package :blackboard_demo)

;;======================================================
;; Translated ROS Messages

(message-type-to-unit-class "gazebo_plugins" "WorldState")
(message-type-to-unit-class "std_msgs" "String")

;;======================================================
;; Classes

(define-unit-class thing ()
  ((location 
    :link (position-3d things-here))))

(define-unit-class object (thing) 
  (gazebo-name 
   name 
   shape 
   color))

(define-unit-class person (thing)
  (name
   face-data
   current-activity))

(define-unit-class position-3d ()
  (x 
   y 
   z
   (things-here 
    :link (thing location)))
  (:dimensional-values
   (x :point x)
   (y :point y)
   (z :point z)))

;; "Events"

(define-unit-class utterance ()
  (sentence
   speaker))

(define-unit-class pointing ()
  (location
   pointer))

;; Interpretations of Language

(define-unit-class command (utterance)
  (verb-int 
   object-int))

(define-unit-class interpretation ()
  (phrase 
   meaning))

