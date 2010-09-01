(in-package :simulation_semantics)

;;=========================================================
;; Simulator Class Definition

(define-unit-class simulator ()
  ((objects :initform (make-hash-table :test 'eq)) ;; Map from object to it's pose (xyz rpy)
   (goal-map :initform (make-hash-table :test 'eq))
   (goals :initform nil) ;; This is a list of goal objects
   (policy-map :initform (make-hash-table :test 'eq))
   (termination-time :initform 10)

   (current-time :initform 0)
   (simulations :initform nil))
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

(defmethod run-simulator ((sim simulator))
  ;; Create the simulator and prepare for execution
  (set-active sim)
  (construct sim)
  (create-simulation sim)
  (start-simulation)
  (loop for obj being the hash-keys of (objects-of sim)
     do (activate obj))

  ;; Main loop of the simulator
  (loop with step = 0.1
     until (should-terminate? sim)
     for last-state = (first (find-instances 'world-state (current-simulation sim) :all))
       ;; why do we need this when we have *current-state* ?
     do (format t "~%~%Time Step ~,3f:~%" (current-time-of sim)) 
       ;(loop for obj being the hash-keys of (policy-map-of sim) using (hash-value policy)
       ;   if policy do 
       ;     (funcall policy obj (current-time-of sim) *current-state*)
       ;   else do 
       ;     (format t "~a is doing nothing.~%" obj))
       (loop for goal in (goals-of sim)
          do (new-seek-goal-policy goal last-state))
       (if last-state 
           (loop for p in (predicates-of last-state) 
              do (format t "~a~%" p))
           (print "NO LAST STATE"))
       ;(if last-state
       ;    (loop for p in (predicates-of last-state)
       ;       when (and (eq (first p) 'dist-to-goal) (< (first (last p)) 0.5))
       ;       do (setf (success-of (current-simulation sim)) t)))
       ;(print *current-msg*)
        (if *current-msg*
           (let* ((ws (translate-world-state *current-msg* (current-simulation sim))))
             (annotate-with-predicates ws)))
       (incf (current-time-of sim) step)

       (format t "Simulator Loop Complete~%~%")
       (sleep step))

  ;; Tear down the simulator
  (loop for obj being the hash-keys of (objects-of sim)
     do (deactivate obj))
  (pause-simulation)
  (destroy sim)
  (setf (current-time-of sim) 0)

  (current-simulation sim) ;; TODO: Would be more useful to return the simulation object for the run
)

(defmethod set-active ((sim simulator))
  (if (not (find-space-instance-by-path '(running-simulators)))
      (make-space-instance '(running-simulators)))
  (add-instance-to-space-instance sim (find-space-instance-by-path '(running-simulators))))

(defmethod should-terminate? ((sim simulator))
  (or (>= (current-time-of sim) (termination-time-of sim))
      (success-of (current-simulation sim))))

(defmethod goal-satisfied ((sim simulator))
  (loop for obj being the hash-keys of (goal-map-of sim) using (hash-value goal)
     if goal do
       (funcall goal obj)))
  
(defmethod construct ((sim simulator))
  "Adds the objects to the simulator, unless they are suppressed."
  (loop for obj being the hash-keys of (objects-of sim) using (hash-value pose)
     do (add-to-world obj pose)
       (sleep 0.5)))

(defmethod destroy ((sim simulator))
  (loop for obj being the hash-keys of (objects-of sim)
     do (remove-from-world obj)
       (sleep 0.5)))

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




;(defmethod suppress ((sim simulator) obj)
;  "Cause the simulator to run without a particular object being created."
;  (setf (gethash obj (suppressed-objects-of sim)) t))

;(defmethod revive ((sim simulator) obj)
;  "Restore a particular object to the simulator."
;  (remhash obj (suppressed-objects-of sim)))

