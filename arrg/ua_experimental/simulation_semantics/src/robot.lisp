(in-package :simulation_semantics)

;;======================================================================
;; This file contains code for controlling the simple robot base
;;======================================================================

(define-unit-class robot (entity)
  ((vel-pub :initform nil)
   (intended-velocity :initform 0))
  )

;;=====================================================================
;; Methods

(defmethod activate ((r robot))
  (declare (ignore r)))

(defmethod deactivate ((r robot))
  (declare (ignore r)))

(defmethod add-to-world ((r robot) pose)
  (call-service "spawn_wubble_base" 'wubble_description-srv:SpawnWubbleBase
                :name (model-name r)
                :initial_pose (make-initial-pose pose)))

(defmethod remove-from-world ((r robot))
  (call-service "delete_wubble_base" 'wubble_description-srv:DeleteWubbleBase
                :name (model-name r)))

(defmethod get-default-predicates ((r robot))
  '(x-pos y-pos z-pos
    moving turning-left turning-right 
    ;x-vel y-vel z-vel 
    vel-mag ;diff-speed
    ;force-mag
    yaw pitch roll))
    ;yaw-vel pitch-vel roll-vel))

;;=====================================================================

;; TODO: Hardcoded for now
(defmethod model-name ((r robot))
  "robot_description")
