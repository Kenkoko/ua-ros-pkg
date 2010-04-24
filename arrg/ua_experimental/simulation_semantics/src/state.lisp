(in-package :simulation_semantics)

(defparameter *current-state* nil)

(defun create-world-space ()
  (if (null (find-space-instance-by-path '(world-state)))
      (make-space-instance '(world-state))))

(defun world-state-handler (msg)
  (loop for object-info across (simulator_experiments-msg:object_info-val msg) 
     do (

(defun subscribe-to-world-state ()
  (subscribe "gazebo_world_state" "simulator_experiments/WorldState" #'world-state-handler))
