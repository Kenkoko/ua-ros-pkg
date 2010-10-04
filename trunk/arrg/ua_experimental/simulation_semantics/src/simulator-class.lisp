(in-package :simulation_semantics)

;;=========================================================
;; Simulator Class Definition

(define-unit-class simulator ()
  ((objects :initform (make-hash-table :test 'eq)) ;; Map from object to it's pose (xyz rpy)
   (goal-map :initform (make-hash-table :test 'eq)) ;; TODO: Get rid of this?
   (goals :initform nil) ;; This is a list of goal objects
   (policy-map :initform (make-hash-table :test 'eq))
   (termination-time :initform 40)

   (current-time :initform 0)
   (current-step :initform 0)
   (start-time :initform 0)

   (simulations :initform nil)
   (traces :initform nil))
)

(defmethod duplicate ((this simulator) duplicate-name)
  (make-instance 'simulator
                 :instance-name duplicate-name
                 :objects (copy-hash-table (objects-of this))
                 :goals (loop for goal in (goals-of this) collect goal)
                 :goal-map (copy-hash-table (goal-map-of this))
                 :policy-map (copy-hash-table (policy-map-of this))
                 :termination-time (termination-time-of this)
                 :current-time 0))

;;===========================================================
;; Simulation class (actually a space class)

(define-space-class simulation ()
  ((simulator :initform nil)
   (success :initform nil))
)

;;=========================================================
;; Simulator Class Methods

(defgeneric run-simulator (sim &key state-callback)
  (:documentation "Please work."))

(defmethod run-simulator ((sim simulator) &key (state-callback nil))
  ;; Create the simulator and prepare for execution
  (set-active sim)

  ;; TODO: Spawning the robot is freezing because gazebo is paused the simulator,
  ;; so moved this here for now. What's a better fix?
  (start-simulation)
  (construct sim)
  ;(create-simulation sim)
  ;(start-simulation)
  (loop for obj being the hash-keys of (objects-of sim)
     do (activate obj))

  ;; Main loop of the simulator
  (setf (start-time-of sim) (ros-time))
  (setf (current-time-of sim) (start-time-of sim))
  (setf (current-step-of sim) 0)

  ;; TODO: Need to look at new signature
  (loop with delay = 0.1
     with curr-state = (wubble_mdp-srv:state-val (call-service "environment/get_state" 'wubble_mdp-srv:GetState))
     with state-history = (list curr-state)
     until (should-terminate? sim curr-state)
     do (format t "~%~%Sim Time ~,3f:~%" (- (current-time-of sim) (start-time-of sim))) 
       (format t  "Step     ~,d:~%" (current-step-of sim)) 
       (format t "State (before action): ~%~a~%" curr-state)

       (setf curr-state (forward-policy))
       (push curr-state state-history)

       (if state-callback
           (funcall state-callback curr-state))

       (setf (current-time-of sim) (ros-time))
       (format t "Simulator Step Complete~%")
       (incf (current-step-of sim))

       (sleep delay)
     finally (setf (traces-of sim) (append (traces-of sim) (list (nreverse state-history)))))

  (format t "~%Simulator Loop Complete~%")

  ;; Tear down the simulator
  (loop for obj being the hash-keys of (objects-of sim)
     do (deactivate obj))
  (destroy sim)
  (pause-simulation)
  
  (first (last (traces-of sim)))
)

(defmethod set-active ((sim simulator))
  (if (not (find-space-instance-by-path '(running-simulators)))
      (make-space-instance '(running-simulators)))
  (add-instance-to-space-instance sim (find-space-instance-by-path '(running-simulators))))

(defun goal-reached? (sim state)
  (every #'identity (loop for goal in (goals-of sim)
                       collect (at-goal? goal state))))

(defun timed-out? (sim)
  (>= (current-step-of sim) (termination-time-of sim)))

(defun should-terminate? (sim state)
  (or (goal-reached? sim state)
      (timed-out? sim)))

(defmethod goal-satisfied ((sim simulator))
  (loop for obj being the hash-keys of (goal-map-of sim) using (hash-value goal)
     if goal do
       (funcall goal obj)))
  
(defmethod construct ((sim simulator))
  "Adds the objects to the simulator, unless they are suppressed."
  (loop for obj being the hash-keys of (objects-of sim) using (hash-value pose)
     do (add-to-world obj pose))
  (loop for goal in (goals-of sim) 
     do (add-to-world goal nil))
  (sleep 0.5))

(defmethod destroy ((sim simulator))
  (loop for obj being the hash-keys of (objects-of sim)
     do (remove-from-world obj))
  (loop for goal in (goals-of sim)
     do (remove-from-world goal)))

(defmethod create-simulation ((sim simulator))
  (let* ((simulation-count (length (simulations-of sim))))
    (if (= 0 simulation-count)
        (make-space-instance (list (instance-name-of sim))))
    (let* ((simn (make-space-instance (list (instance-name-of sim)
                                            (gensym "SIMULATION"))
                                      :class 'simulation
                                      :allowed-unit-classes '(world-state)
                                      :dimensions (dimensions-of 'world-state))))
          (setf (simulator-of simn) sim)
          (push simn (simulations-of sim)))))


(defmethod current-simulation ((sim simulator))
  (let* ((simulations (simulations-of sim)))
    (if (null simulations)
        nil
        (first simulations))))

;;==========================================================================
;; Controlling the Gazebo simulator

(defun start-simulation ()
  (call-service "gazebo/unpause_physics" 'std_srvs-srv:Empty))

;; Note: Gazebo does not publish a time when paused
(defun pause-simulation ()
  (call-service "gazebo/pause_physics" 'std_srvs-srv:Empty))

;;===========================================================================

;; Always moves forward, returns the resulting state
;; TODO: Probably should make a policies file
(defun forward-policy ()
  (wubble_mdp-srv:state-val (call-service "environment/perform_action" 'wubble_mdp-srv:PerformAction
                           :action "forward")))




;(defmethod suppress ((sim simulator) obj)
;  "Cause the simulator to run without a particular object being created."
;  (setf (gethash obj (suppressed-objects-of sim)) t))

;(defmethod revive ((sim simulator) obj)
;  "Restore a particular object to the simulator."
;  (remhash obj (suppressed-objects-of sim)))

