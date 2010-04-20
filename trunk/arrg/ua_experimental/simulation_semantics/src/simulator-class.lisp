(in-package :simulation_semantics)

;;=========================================================

(defparameter *current-simulator* nil)

(defclass simulator ()
  ((objects :initarg :objects :accessor objects :initform '())
   (policy-map :initarg :policy-map :accessor policy-map :initform (make-hash-table :test 'eq))
   (termination-time :initarg :termination-time :accessor termination-time :initform 5)
   ;; "private" members
   (current-time :initform 0 :accessor current-time)
))

(defmethod run-simulator ((sim simulator))
  (start sim)

  (loop with step = 0.1
     until (>= (current-time sim) (termination-time sim))
     do (format t "Time Step ~a:~%" (current-time sim)) 
       (loop for obj being the hash-keys of (policy-map sim) using (hash-value action)
          if action do 
            (format t "~a is doing ~a.~%" obj action)
            (funcall (first action) obj (second action))
          else do 
            (format t "~a is doing nothing.~%" obj))
       (incf (current-time sim) step)
       (sleep step))

  (pause sim)
  (setf (current-time sim) 0)
)

(defmethod construct ((sim simulator))
  (loop for obj in (objects sim)
     do (add-to-world obj)
       (sleep 0.5)))

(defmethod destroy ((sim simulator))
  (loop for obj in (objects sim)
     do (remove-from-world obj)))

(defmethod start ((sim simulator))
  (call-service "start_world" 'std_srvs-srv:Empty))

;; Note: Simulator does not publish a time while the world is paused
(defmethod pause ((sim simulator))
  (call-service "pause_world" 'std_srvs-srv:Empty))

(defmethod reset ((sim simulator))
  (call-service "reset_world" 'std_srvs-srv:Empty)
  (setf (current-time sim) 0))

(defmethod load-simulator ((sim simulator))
  (pause sim)
  (if *current-simulator* 
      (destroy *current-simulator*))
  (setf *current-simulator* sim)
  (construct sim))
      
;;=========================================================
;; Tests

(defparameter *counter* 0)

(defun test-load ()
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
