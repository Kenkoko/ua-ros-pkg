(in-package :simulation_semantics)

(defparameter *current-state* nil)
(defparameter *current-msg* nil)

(defun get-object-state (name state)
  (loop for object-state across (wubble_mdp-msg:object_states-val state)
     if (string-equal name (name-val object-state))
     do (return object-state)))

(defun get-relation (state rel-name &rest obj-names)
  "Returns the relation (msg) that matches rel-name and obj-names (in any order)"
  ;(print obj-names)
  (loop for rel across (wubble_mdp-msg:relations-val state)
     if (and (string-equal (simulator_state-msg:rel_name-val rel) rel-name)
             (= (length (intersection obj-names
                                      (vec-to-list (simulator_state-msg:obj_names-val rel))
                                      :test #'string-equal))
                (length obj-names)))
     do (return rel)))
     
  














;==========================================================
;; OLD OLD OLD















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
                                                 (position-val 
                                                  (pose-val info)))
                                      :orientation (translate-xyzw 
                                                    (orientation-val 
                                                     (pose-val info))))
                 :force (make-instance 'force
                                       :linear (translate-xyz 
                                                (geometry_msgs-msg:force-val 
                                                 (simulator_state-msg:force-val info)))
                                       :torque (translate-xyz 
                                                (geometry_msgs-msg:torque-val 
                                                 (simulator_state-msg:force-val info))))
                 :velocity (make-instance 'velocity
                                          :linear (translate-xyz 
                                                   (geometry_msgs-msg:linear-val 
                                                    (simulator_state-msg:velocity-val info)))
                                          :angular (translate-xyz 
                                                    (geometry_msgs-msg:angular-val 
                                                     (simulator_state-msg:velocity-val info)))
                                          )))

;;============================================================                

;; TODO: Need to handle symmetric also
(defun translate-relation (relation-msg)
  (let* ((name (simulator_state-msg:name-val relation-msg))
         (subject (simulator_state-msg:subject-val relation-msg))
         (object (simulator_state-msg:object-val relation-msg))
         (value (simulator_state-msg:value-val relation-msg))
         (is-numeric? (simulator_state-msg:is_numeric-val relation-msg))
         ;(is-symmetric? (simulator_state-msg:is_symmetric-val relation-msg))
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
                                        (simulator_state-msg:header-val msg))
                                 :space-instances (list simn)
                                 :prev-state last-state
                                 :simulation simn)))
    (loop for relation-msg across (simulator_state-msg:relations-val msg)
       do (loop for rel in (translate-relation relation-msg)
             do (push rel (predicates-of ws))))
    (loop for obj-info across (simulator_state-msg:object_info-val msg)
       for object = (find-object-by-gazebo-name (simulator_state-msg:name-val obj-info))
       when object 
       do (translate-object obj-info object ws)
         (setf *current-state* ws)
       finally (return ws))))

;;============================================================                

(defun world-state-handler (msg)
  (setf *current-msg* msg))
                   
(defun subscribe-to-world-state ()
  (subscribe "gazebo_world_state" "simulator_state/WorldState" #'world-state-handler))
