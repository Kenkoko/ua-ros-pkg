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
                     :self-predicates '(force-mag vel-mag dist-to-goal x-pos diff-speed)
                     :binary-predicates '(dist-between)
                     :mass 20
                     :size 0.4
                     :xyz '(-5.0 0 0.22))))

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
                     :size 0.3
                     :color "WubbleWorld/Blue"
                     :self-predicates '(force-mag vel-mag x-pos)
                     :xyz '(0.5 0 0.3))))

(defun make-static-box ()
  (if (null (find-instance-by-name 'stuck-box 'physical-object))
      (make-instance 'physical-object 
                     :instance-name 'stuck-box
                     :gazebo-name "stuck_box"
                     :color "Gazebo/Red"
                     :self-predicates '(force-mag vel-mag x-pos)
                     :xyz '(0.4 0 0.1)
                     :static? t)))

(defun make-above-sphere ()
  (if (null (find-instance-by-name 'above-sphere 'physical-object))
      (make-instance 'physical-object 
                     :instance-name 'above-sphere
                     :gazebo-name "above_sphere"
                     :shape 'sphere
                     :size 0.1
                     :color "Gazebo/Blue"
                     :self-predicates '(force-mag vel-mag x-pos)
                     :xyz '(0.0 0 0.4)
                     :static? nil)))

;;=====================================================
;; Robot pushing rock up hill

(defun make-sphere ()
  (if (null (find-instance-by-name 'sphere 'physical-object))
      (make-instance 'physical-object 
                     :instance-name 'sphere
                     :gazebo-name "the_sphere"
                     :shape 'sphere
                     :size 0.3
                     :color "Gazebo/Blue"
                     :self-predicates '(force-mag vel-mag x-pos z-pos x-vel z-vel)
                     :xyz '(0.5 0 0.3)
                     :static? nil)))

(defun make-ramp ()
  (if (null (find-instance-by-name 'ramp 'physical-object))
      (make-instance 'physical-object 
                     :instance-name 'ramp
                     :gazebo-name "ramp"
                     :shape 'box
                     :size '(100 10 0.1)
                     :color "Gazebo/Green"
                     :self-predicates nil
                     :xyz '(4 0 0.1)
                     :rpy '(0 -10 0)
                     :static? t)))

(defun make-robot ()
  (if (null (find-instance-by-name 'robot 'physical-object))
      (make-instance 'physical-object 
                     :instance-name 'robot
                     :gazebo-name "base_link"
                     :shape 'box
                     :size '(0.3 0.3 0.2)
                     :color "Wubble/Green"
                     :self-predicates '(force-mag vel-mag x-pos z-pos diff-speed x-vel z-vel)
                     :binary-predicates '(dist-between)
                     :xyz '(0 0 0.1)
                     :rpy '(0 0 0))))

;;=====================================================

(defun init-objects ()
  (make-object-space)
  (make-self)
  (make-above-box)
  (make-front-box)
  (make-static-box)
  (make-sphere)
  (make-above-sphere)
  (make-ramp)
  (make-robot))
  
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
                                     (setf (gethash (first objects) ht) '(apply-force (2000 0 0)))
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

(defun push-sphere-sim ()
  (if (null (find-instance-by-name 'push-sphere 'simulator))
  (let ((objects (list (find-instance-by-name 'self 'physical-object)
                       (find-instance-by-name 'sphere 'physical-object))))
    (make-instance 'simulator 
                   :instance-name 'push-sphere
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

(defun carry-sphere-sim ()
  (if (null (find-instance-by-name 'carry-sphere 'simulator))
  (let ((objects (list (find-instance-by-name 'self 'physical-object) 
                       (find-instance-by-name 'above-sphere 'physical-object))))
    (make-instance 'simulator 
                   :instance-name 'carry-sphere
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

(defun hill-sim ()
  (if (null (find-instance-by-name 'hill 'simulator))
      (let ((objects (list (find-instance-by-name 'sphere 'physical-object)
                           (find-instance-by-name 'ramp 'physical-object))))
        (make-instance 'simulator 
                       :instance-name 'hill
                       :objects objects
                       :policy-map (let ((ht (make-hash-table :test 'eq)))
                                     (setf (gethash 'robot ht) 'demo-policy)
                                     ht)
                       :termination-time 30))))

;;=====================================================

(defun init-simulators ()
  (make-simulator-space)
  (free-sim)
  (push-sim)
  (carry-sim)
  (block-sim)
  (push-sphere-sim)
  (carry-sphere-sim)
  (hill-sim))

;;=====================================================

(defun cleanup ()
  (shutdown-ros-node)
  (delete-blackboard-repository))

(defun uphill ()
  (cleanup)
  (with-ros-node ("simulators")
    (init-robot)
    (init-objects)
    (init-simulators)
    (subscribe-to-world-state)
    (test-simulator (find-instance-by-name 'hill 'simulator))
    (cleanup-robot)))

(defun test-all-simulators ()
  (with-ros-node ("simulators")
    (init-objects)
    (init-simulators)
    (subscribe-to-world-state)

    (test-simulator (find-instance-by-name 'carry-sphere 'simulator))
    (test-simulator (find-instance-by-name 'push-sphere 'simulator))

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
