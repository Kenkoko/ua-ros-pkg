(in-package :simulation_semantics)

;; NB: The entities computed here are really features, not just predicates

;;=================================================================
;; Predicate-related methods of world-state class

(defmethod annotate-with-predicates ((ws world-state))
  (let* ((last-ws (if (prev-state-of ws) (prev-state-of ws) ws))
         (predicates (append (loop for obj-state in (objects-of ws)
                                for obj-last-state in (objects-of last-ws)
                                append (compute-my-predicates obj-state obj-last-state))
                             (compute-binary-predicates (objects-of ws)))))
    (setf (predicates-of ws) predicates)))

(defmethod print-predicates ((ws world-state))
  (format t "Predicates of ~a at time ~d:~%" (first (space-instances-of ws)) (time-of ws))
  (loop for pred in (predicates-of ws)
     do (format t "~t~a~%" pred)))

;;=================================================================

(defun compute-binary-predicates (objects)
  "Generate all pairs of objects x and y, and compute 
   the predicates p(x,y) for each of them"
  (if (> (length objects) 1)
      (let* ((prm (make-permutator objects objects))
             (pairs (loop for x = (funcall prm)
                       until (null x)
                       unless (eq (first x) (second x))
                       collect x)))
        (loop for pair in pairs 
           for self = (object-of (first pair))
           for other = (object-of (second pair))
           when (binary-predicates-of self)
           append (loop for bpred in (binary-predicates-of self)
                     collect (list bpred (gazebo-name-of self) (gazebo-name-of other)  
                                   (funcall bpred (first pair) (second pair))))))))
         
;; This sort of needs access to the simulator too, huh? - goals, etc.
(defmethod compute-my-predicates ((my-state object-state) (my-last-state object-state))
  (let* ((me (object-of my-state)))
    (loop for pred in (self-predicates-of me) 
       collect (list pred (gazebo-name-of me) (funcall pred my-state my-last-state)))))
              
(defun print-last-predicates (sim)
  (loop for state in (get-states-from-last-run sim)
     do (print-predicates state)))

;;==================================================================

(defun match-predicate (target-pred pred-name objects)
  ;(format t "~a ~a ~a~%" target-pred pred-name objects)
  (if (eq (first target-pred) pred-name)
      (let ((match-index (search objects (rest target-pred) :test 'equal)))
        (if match-index
            (zerop match-index)))))

(defun plot-by-predicate (sims pred &rest objects)
  "Plots each predicate on a separate chart, across simulators."
  (call-service "plot" 'plotter-srv:Plot
                :plots (loop for sim in sims collect 
                            (loop with states = (get-states-from-last-run sim)
                               with start-time = (time-of (first states))
                               with x = (make-array (length states) :adjustable t :fill-pointer 0)
                               with y = (make-array (length states) :adjustable t :fill-pointer 0)
                               for state in states
                               for predicates = (predicates-of state)
                               do ;(format t "~a~%" state) 
                                 (loop for p in predicates 
                                     when (match-predicate p pred objects)
                                     do (vector-push-extend (- (time-of state) start-time) x)
                                       (vector-push-extend (first (last p)) y))
                               finally (return (make-msg "plotter/PlotData"
                                                         (name) (format nil "Predicate (~a ~a) of ~a simulator"
                                                                        pred objects sim)
                                                         (x_label) "time"
                                                         (y_label) (format nil "(~a ~a)"
                                                                           pred objects)
                                                         (x_data) x
                                                         (y_data) y))))))

(defun plot-by-simulator (sims preds &rest objects)
  "Plots each predicate on a separate chart, across simulators."
  (loop for sim in sims 
     do (call-service "plot" 'plotter-srv:Plot
                      :plots (loop for pred in preds collect 
                                  (loop with states = (get-states-from-last-run sim)
                                     with start-time = (time-of (first states))
                                     with x = (make-array (length states) :adjustable t :fill-pointer 0)
                                     with y = (make-array (length states) :adjustable t :fill-pointer 0)
                                     for state in states
                                     for predicates = (predicates-of state)
                                     do ;(format t "~a~%" state) 
                                       (loop for p in predicates 
                                    when (match-predicate p pred objects)
                                          do (vector-push-extend (- (time-of state) start-time) x)
                                            (vector-push-extend (first (last p)) y))
                                     finally (return (make-msg "plotter/PlotData"
                                                               (name) (format nil "Predicate (~a ~a) of ~a simulator"
                                                                              pred objects sim)
                                                               (x_label) "time"
                                                               (y_label) (format nil "(~a ~a)"
                                                                                 pred objects)
                                                               (x_data) x
                                                               (y_data) y)))))))

;;==================================================================
;; Self-Predicates

(defun force-mag (os last-os)
  (declare (ignore last-os))
  (sqrt (sum-of-squares (as-list (linear-of (force-of os))))))
       
(defun vel-mag (os last-os)
  (declare (ignore last-os))
  (sqrt (sum-of-squares (as-list (linear-of (velocity-of os))))))

(defun x-pos (os last-os)
  (declare (ignore last-os))
  (x-of (position-of (pose-of os))))

(defun y-pos (os last-os)
  (declare (ignore last-os))
  (y-of (position-of (pose-of os))))

(defun z-pos (os last-os)
  (declare (ignore last-os))
  (z-of (position-of (pose-of os))))

;; Hardcoded point for now
(defun dist-to-goal (os last-os)
  (declare (ignore last-os))
  (distance (position-of (pose-of os)) (make-instance 'xyz :x 4 :y 0 :z 0)))
          
(defun x-vel (os last-os)
  (declare (ignore last-os))
  (x-of (linear-of (velocity-of os))))

(defun z-vel (os last-os)
  (declare (ignore last-os))
  (z-of (linear-of (velocity-of os))))

(defun int-vel (os last-os)
  (declare (ignore last-os))
  (first (intended-velocity-of (object-of os))))

;;==================================================================
;; Differential Self-Predicates

(defun diff-speed (os last-os)
  (distance (position-of (pose-of os)) (position-of (pose-of last-os))))
        
;;==================================================================
;; Binary Predicates

(defun dist-between (self-state other-state)
  (distance (position-of (pose-of self-state)) (position-of (pose-of other-state))))

;;==================================================================

(defun sum-of-squares (numbers)
  (loop for x in numbers summing (expt x 2)))

;(defmethod distance ((self physical-object) (other physical-object))
;  (distance (position-of (pose-of self)) (position-of (pose-of other))))