(in-package :simulation_semantics)

;;=========================================================
;; Simulator Class Definition

(define-unit-class simulator ()
  ((objects :initform '())
   (goal-map :initform (make-hash-table :test 'eq))
   (policy-map :initform (make-hash-table :test 'eq))
   (termination-time :initform 5)
   ;; "private" members
   (current-time :initform 0)
   ;; links
   (simulations 
    :link (simulation simulator :singular t)))

  (:initial-space-instances (simulator-library))
)

;;=========================================================
;; Simulator Class Methods

;; This is a bit harsher on the simulator (kills everything when it is done)
(defmethod run-simulator ((sim simulator))
  (add-instance-to-space-instance sim (find-space-instance-by-path '(running-simulators)))

  (reset sim)

  (construct sim)
  
  (create-simulation sim)

  (start sim)

  (loop for obj in (objects-of sim)
     do (activate obj))

  (loop with step = 0.1
     until (>= (current-time-of sim) (termination-time-of sim))
     do (format t "Time Step ~a:~%" (current-time-of sim)) 
       (loop for obj being the hash-keys of (policy-map-of sim) using (hash-value policy)
          if policy do 
            (funcall policy obj (current-time-of sim) nil)
            ;(format t "~a is doing ~a.~%" obj action)
            ;(funcall (first action) obj (second action))
          else do 
            (format t "~a is doing nothing.~%" obj))
       (incf (current-time-of sim) step)
       (sleep step))

  (loop for obj in (objects-of sim)
     do (deactivate obj))

  (pause sim)
  (destroy sim)
  (setf (current-time-of sim) 0)

  (remove-instance-from-space-instance sim (find-space-instance-by-path '(running-simulators)))
)

(defmethod construct ((sim simulator))
  (loop for obj in (objects-of sim)
     do (add-to-world obj)
       (sleep 0.5)))

(defmethod destroy ((sim simulator))
  (loop for obj in (objects-of sim)
     do (remove-from-world obj)
       (sleep 0.5)))

(defmethod start ((sim simulator))
  (call-service "start_world" 'std_srvs-srv:Empty))

;; Note: Gazebo does not publish a time when paused
(defmethod pause ((sim simulator))
  (call-service "pause_world" 'std_srvs-srv:Empty))

(defmethod reset ((sim simulator))
  (call-service "reset_world" 'std_srvs-srv:Empty)
  (setf (current-time-of sim) 0))

#+ignore(defmethod load-simulator ((sim simulator))
  (pause sim)
  (if *current-simulator* 
      (destroy *current-simulator*))
  (setf *current-simulator* sim)
  (construct sim))
      




;;=========================================================
;; Tests

(defparameter *counter* 0)

#+ignore(defun test-load ()
  (with-ros-node ("tester")
    (let* ((obj1 (make-instance 'physical-object :gazebo-name "bottom_box" :xyz '(0 0 0.2)))
           (obj2 (make-instance 'physical-object :gazebo-name "top_box" :xyz '(0 0 0.6)))
           (pmap (let ((ht (make-hash-table :test 'eq)))
                   (setf (gethash obj1 ht) '(apply-force (100 0 0)))
                   (setf (gethash obj2 ht) nil)
                   ht))
           (sim (make-instance 'simulator 
                               :objects (list obj1 obj2)
                               :policy-map pmap)))
      (load-simulator sim)
      (run-simulator sim)
      (sleep 2)
      (destroy sim))))

(defun test-construct ()
  (with-ros-node ("tester")
    (let ((sim (make-instance 'simulator 
                              :objects (list (make-instance 'physical-object 
                                                            :gazebo-name "bottom_box"
                                                            :xyz '(0 0 0.2))
                                             (make-instance 'physical-object 
                                                            :gazebo-name "top_box"
                                                            :xyz '(0 0 0.6))))))
      (construct sim)
      (wait-duration 10)
      (reset sim)
      (wait-duration 10)
      (destroy sim))))

(defun test-object-creation ()
  (with-ros-node ("tester")
    (if (wait-for-service "/add_model" 10)
        (let ((obj (make-instance 'physical-object
                                  :gazebo-name (concatenate 'string
                                                            "blue_box_"
                                                            (format nil "~a" (incf *counter*)))
                                  :xyz (list 0 0 (* 0.5 *counter*)))))
          (format t "SERVICE OK, OBJECT CREATED")
          (format t "~a~%" (add-to-world obj))
          (format t "WAITING 10 Seconds...")
          (wait-duration 10)
          (format t "DELETING OBJECT...")
          (format t "~a~%" (remove-from-world obj))
          (format t "Done.~%"))
        (ros-warn nil "Timed out waiting for service add_model"))))
