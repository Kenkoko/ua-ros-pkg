(in-package :simulation_semantics)

(defparameter *current-state* nil)
(defparameter *current-msg* nil)

;;============================================================                

(defun get-states-from-last-run (simulator-name)
  "Convenience function for gathering states from most recent run of a simulator"
  (reverse (find-instances 'world-state 
                           (first (simulations-of (find-instance-by-name simulator-name 'simulator)))
                           :all)))

(defun get-states (simulation)
  (reverse (find-instances 'world-state simulation :all)))

;;============================================================                

(defun find-object-by-gazebo-name (gazebo-name)
  (let* ((result (find-instances '(entity :plus-subclasses)
                                 '(object-library)
                                 (list 'is-equal 
                                       'gazebo-name 
                                       gazebo-name))))
    (if result (first result) nil)))

;;============================================================                

(defun translate-object (info object ws)
  (make-instance 'object-state
                 :world ws
                 :object object
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

;;============================================================                

;; TODO: Need to handle symmetric also
(defun translate-relation (relation-msg)
  (let* ((name (simulator_experiments-msg:name-val relation-msg))
         (subject (simulator_experiments-msg:subject-val relation-msg))
         (object (simulator_experiments-msg:object-val relation-msg))
         (value (simulator_experiments-msg:value-val relation-msg))
         (is-numeric? (simulator_experiments-msg:is_numeric-val relation-msg))
         (is-symmetric? (simulator_experiments-msg:is_symmetric-val relation-msg))
         (result nil))
    (if is-numeric?
        (push (list name subject object value) result)
        (push (list name subject object (not (zerop value))) result))
    #+ignore(if is-symmetric?
        (if is-numeric?
            (push (list name object subject value) result)
            (push (list name object subject (not (zerop value))) result)))
))
        
;;============================================================                

(defun translate-world-state (msg simn)
  ;(print msg)
  (let* ((states (find-instances 'world-state simn :all))
         (last-state (if states (first states) nil))
         (ws (make-instance 'world-state 
                                 :time (roslib-msg:stamp-val
                                        (simulator_experiments-msg:header-val msg))
                                 :space-instances (list simn)
                                 :prev-state last-state
                                 :simulation simn)))
    (loop for relation-msg across (simulator_experiments-msg:relations-val msg)
       do (loop for rel in (translate-relation relation-msg)
             do (push rel (predicates-of ws))))
    (loop for obj-info across (simulator_experiments-msg:object_info-val msg)
       for object = (find-object-by-gazebo-name (simulator_experiments-msg:name-val obj-info))
       when object 
       do (translate-object obj-info object ws)
         (setf *current-state* ws)
       finally (return ws))))

;;============================================================                

(defun world-state-handler (msg)
  (setf *current-msg* msg))
                   
(defun subscribe-to-world-state ()
  (subscribe "gazebo_world_state" "simulator_experiments/WorldState" #'world-state-handler))
