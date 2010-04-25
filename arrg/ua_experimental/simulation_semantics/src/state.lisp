(in-package :simulation_semantics)

(defparameter *current-state* nil)

(defun create-world-space ()
  (if (null (find-space-instance-by-path '(world-state)))
      (make-space-instance '(world-state))))

(defun translate-xyz (msg)
  (make-instance 'xyz 
                 :x (geometry_msgs-msg:x-val msg)
                 :y (geometry_msgs-msg:y-val msg)
                 :z (geometry_msgs-msg:z-val msg)))

(defun translate-xyzw (msg)
  (make-instance 'quaternion 
                 :x (geometry_msgs-msg:x-val msg)
                 :y (geometry_msgs-msg:y-val msg)
                 :z (geometry_msgs-msg:z-val msg)
                 :w (geometry_msgs-msg:w-val msg)))
                
(defun translate-object (info ws)
  (make-instance 'object-state
                 :world ws
                 :object (find-instances 'physical-object
                                         '(object-library)
                                         (list 'is-equal 
                                               'gazebo-name 
                                               (simulator_experiments-msg:name-val info)))
                 :pose (make-instance 'pose
                                      :position (translate-xyz
                                                 (geometry_msgs-msg:position-val 
                                                  (simulator_experiments-msg:pose-val info)))
                                      :orientation (translate-xyzw 
                                                    (geometry_msgs-msg:orientation-val 
                                                     (simulator_experiments-msg:pose-val info))))
                 :force (make-instance 'force
                                       :linear (translate-xyz 
                                                (geometry_msgs-msg:force-val 
                                                 (simulator_experiments-msg:force-val info)))
                                       :torque (translate-xyz 
                                                (geometry_msgs-msg:torque-val 
                                                 (simulator_experiments-msg:force-val info))))
                 :velocity (make-instance 'velocity
                                          :linear (translate-xyz 
                                                   (geometry_msgs-msg:linear-val 
                                                    (simulator_experiments-msg:velocity-val info)))
                                          :angular (translate-xyz 
                                                    (geometry_msgs-msg:angular-val 
                                                     (simulator_experiments-msg:velocity-val info)))
                                          )))

(defun translate-world-state (msg simn)
  (loop with ws = (make-instance 'world-state 
                                 :time (roslib-msg:stamp-val
                                        (simulator_experiments-msg:header-val msg))
                                 :space-instances (list simn))
     for obj-info across (simulator_experiments-msg:object_info-val msg)
     do (translate-object obj-info ws)
     finally (return ws)))

(defun world-state-handler (msg)
  (loop for simr in (find-instances 'simulator '(running-simulators) :all)
     for simn = (current-simulation simr)
     for ws = (translate-world-state msg simn)
     do (annotate-with-predicates ws)))
              
(defun subscribe-to-world-state ()
  (subscribe "gazebo_world_state" "simulator_experiments/WorldState" #'world-state-handler))
