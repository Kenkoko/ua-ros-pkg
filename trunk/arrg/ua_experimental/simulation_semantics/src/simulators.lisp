(in-package :simulation_semantics)

;;=====================================================
;; Objects

(defun make-object-space ()
  (if (null (find-space-instance-by-path '(object-library)))
      (make-space-instance '(object-library)
                           :dimensions (dimensions-of 'physical-object))))

(defun make-self ()
  (if (null (find-instance-by-name 'self 'physical-object))
      (make-instance 'physical-object 
                     :instance-name 'self
                     :gazebo-name "self"
                     :color "Gazebo/Green"
                     :self-predicates '(force-mag vel-mag dist-to-goal x-pos)
                     :xyz '(0 0 0.1))))

(defun make-above-box ()
  (if (null (find-instance-by-name 'above-box 'physical-object))
      (make-instance 'physical-object 
                     :instance-name 'above-box
                     :gazebo-name "above_box"
                     :self-predicates '(force-mag vel-mag x-pos)
                     :xyz '(0 0 0.4))))

(defun make-front-box ()
  (if (null (find-instance-by-name 'front-box 'physical-object))
      (make-instance 'physical-object 
                     :instance-name 'front-box
                     :gazebo-name "front_box"
                     :self-predicates '(force-mag vel-mag x-pos)
                     :xyz '(0.4 0 0.1))))

(defun make-static-box ()
  (if (null (find-instance-by-name 'stuck-box 'physical-object))
      (make-instance 'physical-object 
                     :instance-name 'stuck-box
                     :gazebo-name "stuck_box"
                     :color "Gazebo/Red"
                     :self-predicates '(force-mag vel-mag x-pos)
                     :xyz '(0.4 0 0.1)
                     :static? t)))

(defun init-objects ()
  (make-object-space)
  (make-self)
  (make-above-box)
  (make-front-box)
  (make-static-box))
  
;;=====================================================
;; Simulators

(defun make-simulator-space ()
  (if (null (find-space-instance-by-path '(simulator-library)))
      (progn (make-space-instance '(simulator-library))
             (make-space-instance '(running-simulators)))))

(defun free-sim ()
  (if (null (find-instance-by-name 'free 'simulator))
      (let ((objects (list (find-instance-by-name 'self 'physical-object))))
        (make-instance 'simulator 
                       :instance-name 'free
                       :objects objects
                       :policy-map (let ((ht (make-hash-table :test 'eq)))
                                     (setf (gethash (first objects) ht) '(apply-force (200 0 0)))
                                     ht)))))

(defun push-sim ()
  (if (null (find-instance-by-name 'push 'simulator))
  (let ((objects (list (find-instance-by-name 'self 'physical-object)
                       (find-instance-by-name 'front-box 'physical-object))))
    (make-instance 'simulator 
                   :instance-name 'push
                   :objects objects
                   :policy-map (let ((ht (make-hash-table :test 'eq)))
                                 (setf (gethash (first objects) ht) '(apply-force (200 0 0)))
                                 ht)))))

(defun carry-sim ()
  (if (null (find-instance-by-name 'carry 'simulator))
  (let ((objects (list (find-instance-by-name 'self 'physical-object) 
                       (find-instance-by-name 'above-box 'physical-object))))
    (make-instance 'simulator 
                   :instance-name 'carry
                   :objects objects
                   :policy-map (let ((ht (make-hash-table :test 'eq)))
                                 (setf (gethash (first objects) ht) '(apply-force (200 0 0)))
                                 ht)))))

(defun block-sim ()
  (if (null (find-instance-by-name 'block 'simulator))
  (let ((objects (list (find-instance-by-name 'self 'physical-object) 
                       (find-instance-by-name 'stuck-box 'physical-object))))
    (make-instance 'simulator 
                   :instance-name 'block
                   :objects objects
                   :policy-map (let ((ht (make-hash-table :test 'eq)))
                                 (setf (gethash (first objects) ht) '(apply-force (200 0 0)))
                                 ht)))))

(defun init-simulators ()
  (make-simulator-space)
  (free-sim)
  (push-sim)
  (carry-sim)
  (block-sim))

;;=====================================================

(defun cleanup ()
  (shutdown-ros-node)
  (delete-blackboard-repository))

(defun test-all-simulators ()
  (with-ros-node ("simulators")
    (init-objects)
    (init-simulators)
    (subscribe-to-world-state)
    (test-simulator (find-instance-by-name 'free 'simulator))
    (test-simulator (find-instance-by-name 'push 'simulator))
    (test-simulator (find-instance-by-name 'carry 'simulator))
    (test-simulator (find-instance-by-name 'block 'simulator))))

(defun stress-test ()
  (dotimes (x 20)
    (test-all-simulators)))

(defun test-simulator (sim)
    ;(load-simulator sim)
    (run-simulator sim)
    (sleep 2)
    (destroy sim)
    (sleep 2))
