(in-package :simulation_semantics)

;;=====================================================
;; Objects

(defparameter *object-hash* (make-hash-table))

(defun make-self ()
  (make-instance 'physical-object 
                 :gazebo-name "self"
                 :xyz '(0 0 0.1)))

(defun make-above-box ()
  (make-instance 'physical-object 
                 :gazebo-name "above_box"
                 :xyz '(0 0 0.4)))

(defun make-front-box ()
  (make-instance 'physical-object 
                 :gazebo-name "front_box"
                 :xyz '(0.4 0 0.1)))

(defun make-static-box ()
  (make-instance 'physical-object 
                 :gazebo-name "stuck_box"
                 :xyz '(0.4 0 0.1)
                 :is-static t))

(defun init-objects ()
  (setf (gethash "self" *object-hash*) (make-self))
  (setf (gethash "above_box" *object-hash*) (make-above-box))
  (setf (gethash "front_box" *object-hash*) (make-front-box))
  (setf (gethash "stuck_box" *object-hash*) (make-static-box)))
  
;;=====================================================
;; Simulators

(defun free-sim ()
  (let ((objects (list (gethash "self" *object-hash*))))
    (make-instance 'simulator 
                   :objects objects
                   :policy-map (let ((ht (make-hash-table :test 'eq)))
                                 (setf (gethash (first objects) ht) '(apply-force (200 0 0)))
                                 ht))))

(defun push-sim ()
  (let ((objects (list (gethash "self" *object-hash*) (gethash "front_box" *object-hash*))))
    (make-instance 'simulator 
                   :objects objects
                   :policy-map (let ((ht (make-hash-table :test 'eq)))
                                 (setf (gethash (first objects) ht) '(apply-force (200 0 0)))
                                 ht))))

(defun carry-sim ()
  (let ((objects (list (gethash "self" *object-hash*) (gethash "above_box" *object-hash*))))
    (make-instance 'simulator 
                   :objects objects
                   :policy-map (let ((ht (make-hash-table :test 'eq)))
                                 (setf (gethash (first objects) ht) '(apply-force (200 0 0)))
                                 ht))))

(defun block-sim ()
  (let ((objects (list (gethash "self" *object-hash*) (gethash "stuck_box" *object-hash*))))
    (make-instance 'simulator 
                   :objects objects
                   :policy-map (let ((ht (make-hash-table :test 'eq)))
                                 (setf (gethash (first objects) ht) '(apply-force (200 0 0)))
                                 ht))))

;;=====================================================

(defun test-simulator (sim)
    (load-simulator sim)
    (run-simulator sim)
    (sleep 2)
    (destroy sim)
    (sleep 2))
