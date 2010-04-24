(in-package :simulation_semantics)

(defun create-world-space ()
  (if (null (find-space-instance-by-path '(world-state)))
      (make-space-instance '(world-state))))

(defun world-state-handler (msg)
  (loop for object-info in (simulator_experiments-msg:object_info-val msg)
     do (print object-info)))

(defun subscribe-to-world-state ()
  (subscribe "gazebo_world_state" "simulator_experiments/WorldState" #'world-state-handler))
