(in-package :simulation_semantics)

(defparameter *binary-predicate-list* 
  '(dist-between rel-angle))

;; TODO: This whole file is obselete now, sigh.

;;=================================================================
;; Predicate-related methods of world-state class

(defmethod annotate-with-predicates ((ws world-state))
  (format t "ANNOTATE CALLED with state ~a~%" ws)
  (let* ((last-ws (if (prev-state-of ws) (prev-state-of ws) ws))
         (predicates (append (loop for obj-state in (objects-of ws)
                                for obj-last-state in (objects-of last-ws)
                                append (compute-my-predicates obj-state obj-last-state))
                             (compute-binary-predicates (objects-of ws)))))
    (setf (predicates-of ws) (append (predicates-of ws) predicates))
    (loop for goal in (goals-of (simulator-of (simulation-of ws)))
       do (format t "GOAL DEBUG: ~a ~a~%" goal ws)
         (nconc (predicates-of ws) (distance-to-goal goal ws)))))

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
                     collect (list (string-capitalize (string bpred)) (gazebo-name-of self) (gazebo-name-of other)  
                                   (funcall bpred (first pair) (second pair))))))))
         
;; This sort of needs access to the simulator too, huh? - goals, etc.
(defmethod compute-my-predicates ((my-state object-state) (my-last-state object-state))
  (let* ((me (object-of my-state)))
    (loop for pred in (get-self-predicates me)
       collect (list (string-capitalize (string pred)) (gazebo-name-of me) (funcall pred my-state my-last-state)))))
              
(defun print-last-predicates (sim)
  (loop for state in (get-states-from-last-run sim)
     do (print-predicates state)))

(defun get-self-predicates (entity)
  (if (eq (self-predicates-of entity) :all)
      (get-default-predicates entity)
      (self-predicates-of entity)))

;;==================================================================

(defun extract-value (pred)
  (let* ((v (last pred)))
    (while (listp v)
      (setf v (first v)))
    v))

;;==================================================================
;; Self-Predicates

;;--------------------------------------------------------------------
;; PROPOSITIONS

;; TODO: Have not checked these dead zones at all

(defun moving (os last-os)
  (> (vel-mag os last-os) 0.05))

(defun turning-left (os last-os)
  (< (yaw-vel os last-os) -0.05))

(defun turning-right (os last-os)
  (> (yaw-vel os last-os) 0.05))

;;--------------------------------------------------------------------
;; REAL-VALUED

;; FORCE

;; TODO: Something is wrong on the gazebo end so this is always zero
(defun force-mag (os last-os)
  (declare (ignore last-os))
  (sqrt (sum-of-squares (as-list (linear-of (force-of os))))))
       
;; POSITION

(defun x-pos (os last-os)
  (declare (ignore last-os))
  (x-of (position-of (pose-of os))))

(defun y-pos (os last-os)
  (declare (ignore last-os))
  (y-of (position-of (pose-of os))))

(defun z-pos (os last-os)
  (declare (ignore last-os))
  (z-of (position-of (pose-of os))))

;; TRANSLATIONAL VELOCITY
          
(defun x-vel (os last-os)
  (declare (ignore last-os))
  (x-of (linear-of (velocity-of os))))

(defun y-vel (os last-os)
  (declare (ignore last-os))
  (y-of (linear-of (velocity-of os))))

(defun z-vel (os last-os)
  (declare (ignore last-os))
  (z-of (linear-of (velocity-of os))))

(defun int-vel (os last-os)
  (declare (ignore last-os))
  (first (intended-velocity-of (object-of os))))

(defun vel-mag (os last-os)
  (declare (ignore last-os))
  (sqrt (sum-of-squares (as-list (linear-of (velocity-of os))))))

;; ROTATION

(defun yaw (os last-os)
  (declare (ignore last-os))
  (get-yaw (orientation-of (pose-of os))))

(defun pitch (os last-os)
  (declare (ignore last-os))
  (get-pitch (orientation-of (pose-of os))))

(defun roll (os last-os)
  (declare (ignore last-os))
  (get-roll (orientation-of (pose-of os))))
  
;; ROTATIONAL VELOCITY

(defun yaw-vel (os last-os)
  (declare (ignore last-os))
  (x-of (angular-of (velocity-of os))))

(defun pitch-vel (os last-os)
  (declare (ignore last-os))
  (y-of (angular-of (velocity-of os))))

(defun roll-vel (os last-os)
  (declare (ignore last-os))
  (z-of (angular-of (velocity-of os))))

(defun int-yaw-vel (os last-os)
  (declare (ignore last-os))
  (second (intended-velocity-of (object-of os))))

;; This should be obselete now
;(defun dist-to-goal (os last-os)
;  (declare (ignore last-os))
;  (distance (position-of (pose-of os)) (make-instance 'xyz :x 6 :y 0 :z 0)))


;;==================================================================
;; Differential Self-Predicates

;; TODO: Something off about this, probably need to multiply by sample rate to get 
;; m/s instead of m/sample interval
(defun diff-speed (os last-os)
  (distance (position-of (pose-of os)) (position-of (pose-of last-os))))
        
;;==================================================================
;; Binary Predicates

(defun dist-between (self-state other-state)
  (distance (position-of (pose-of self-state)) (position-of (pose-of other-state))))

(defun rel-angle (self-state other-state)
  (relative-angle (get-abs-yaw (orientation-of (pose-of self-state)))
                  (x-of (position-of (pose-of self-state)))
                  (y-of (position-of (pose-of self-state)))
                  (x-of (position-of (pose-of other-state)))
                  (y-of (position-of (pose-of other-state)))))

;;==================================================================

(defun sum-of-squares (numbers)
  (loop for x in numbers summing (expt x 2)))

;(defmethod distance ((self physical-object) (other physical-object))
;  (distance (position-of (pose-of self)) (position-of (pose-of other))))