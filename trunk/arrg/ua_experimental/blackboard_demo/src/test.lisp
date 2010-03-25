(in-package :blackboard_demo)

;(translate-unit-class (find-class 'gazebo_plugins-msg:<WorldState>)
;(message-type-to-unit-class "gazebo_plugins" "WorldState")

(defparameter *msg* nil)

;; Initial translation of the WorldState message class into a GBBopen class
(defun test ()
  (with-ros-node ("test" :spin t)
    (subscribe "gazebo_world_state" "gazebo_plugins/WorldState" #'translate-msg)))

(defun update-world-state (state-msg)
  (let* ((ui (translate-msg state-msg)))
    (describe-instance ui)))
