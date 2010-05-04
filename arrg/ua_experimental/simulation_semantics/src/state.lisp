(in-package :simulation_semantics)

(defparameter *current-state* nil)

;; Don't think we need this anymore
;(defun create-world-space ()
;  (if (null (find-space-instance-by-path '(world-state)))
;      (make-space-instance '(world-state))))

;;============================================================                

(defun get-states-from-last-run (simulator-name)
  "Convenience function for gathering states from most recent run of a simulator"
  (reverse (find-instances 'world-state 
                           (first (simulations-of (find-instance-by-name simulator-name 'simulator)))
                           :all)))

;;============================================================                

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
  (loop with states = (find-instances 'world-state simn :all)
     with last-state = (if states (first states) nil)
     with ws = (make-instance 'world-state 
                                 :time (roslib-msg:stamp-val
                                        (simulator_experiments-msg:header-val msg))
                                 :space-instances (list simn)
                                 :prev-state last-state)
     for obj-info across (simulator_experiments-msg:object_info-val msg)
     do (translate-object obj-info ws)
     finally (return ws)))

;;============================================================                

(defun world-state-handler (msg)
  (loop for simr in (find-instances 'simulator '(running-simulators) :all)
     for simn = (current-simulation simr) ;; This is sometimes null - possible timing issue?
     if (null simn)
     do (ros-error "TEST" "SIMULATION IS NULL IN WS HANDLER")
       (format t "SIMULATION IS NULL for simulator ~a~%" simr)
     else do (let* ((ws (translate-world-state msg simn)))
               (annotate-with-predicates ws))))
                   
(defun subscribe-to-world-state ()
  (subscribe "gazebo_world_state" "simulator_experiments/WorldState" #'world-state-handler))
