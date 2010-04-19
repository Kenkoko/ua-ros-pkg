(in-package :simulation_semantics)

;;=========================================================

(defclass simulator ()
  ((objects :initarg objects :accessor objects :initform '())
   (policy-map :initarg policy-map :accessor policy-map :initform (make-hash-table))
   (termination-predicate :initarg termination-predicate :accessor termination-predicate 
                          :initform (lambda (arg) t))))

(defmethod run-simulator ((sim simulator))
  ;; Send the objects to simulation manager
  ;; Loop on a timer, at each time step run the policy for each object
  ;; When termination condition is true (make it a function), terminate
)

;;=========================================================

(defun make-it-so ()
  (with-ros-node ("tester")
    (if (wait-for-service "/add_model" 10)
        (let ((obj (make-instance 'physical-object
                                  :gazebo-name (concatenate 'string
                                                            "blue_box_"
                                                            (format nil "~a" (random 100)))
                                  :xyz '(0 0 0.5))))
          (format t "SERVICE OK, OBJECT CREATED")
          (format t "~a~%" (add-to-world obj))
          (format t "WAITING 2 Seconds...")
          (wait-duration 2)
          (format t "DELETING OBJECT...")
          (format t "~a~%" (remove-from-world obj))
          (format t "Done.~%"))
        (ros-warn nil "Timed out waiting for service add_model"))))
