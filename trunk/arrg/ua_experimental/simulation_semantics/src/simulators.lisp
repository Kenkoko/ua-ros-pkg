(in-package :simulation_semantics)

;;=====================================================
;; Objects

(defun make-self ()
  (if (null (find-instance-by-name 'self 'physical-object))
      (make-instance 'physical-object 
                     :instance-name 'self
                     :gazebo-name "self"
                     :color "Gazebo/Green"
                     :self-predicates '(force-mag vel-mag dist-to-goal x-pos diff-speed)
                     :binary-predicates '(dist-between)
                     ;:mass 0.2 
                     :size 0.2
                     :xyz '(0 0 0.1))))

(defun make-above-box ()
  (if (null (find-instance-by-name 'above-box 'physical-object))
      (make-instance 'physical-object 
                     :instance-name 'above-box
                     :gazebo-name "above_box"
                     :mass 5.0
                     :self-predicates '(force-mag vel-mag x-pos)
                     :xyz '(0 0 0.4))))

(defun make-front-box ()
  (if (null (find-instance-by-name 'front-box 'physical-object))
      (make-instance 'physical-object 
                     :instance-name 'front-box
                     :gazebo-name "front_box"
                     :size 0.3
                     :mass 1000.0
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
                     :xyz '(1.0 0 0.1)
                     :size '(0.2 0.6 0.2)
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
                     :size '(100 100 0.1)
                     :color "Gazebo/Green"
                     :self-predicates nil
                     :xyz '(4 0 0.1)
                     :rpy '(0 -10 0)
                     :static? t)))

;;=====================================================

(defun make-robot ()
  (if (null (find-instance-by-name 'robot 'robot))
      (make-instance 'robot 
                     :instance-name 'robot
                     :gazebo-name "base_footprint"
                     ;:size '(0.3 0.3 0.2)
                     :self-predicates '(dist-to-goal
                                        x-pos z-pos 
                                        int-vel vel-mag diff-speed x-vel z-vel
                                        force-mag)
                     :binary-predicates '(dist-between))))
                     ;:xyz '(0 0 0.1)
                     ;:rpy '(0 0 0))))

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
      (let ((objects (list ;(find-instance-by-name 'self 'physical-object))))
                      (find-instance-by-name 'robot 'robot))))
        (make-instance 'simulator 
                       :instance-name 'free
                       :objects objects
                       :policy-map (let ((ht (make-hash-table :test 'eq)))
                                     (setf (gethash (first objects) ht) 'move-forward-policy)
                                     ht)))))

(defun push-sim ()
  (if (null (find-instance-by-name 'push 'simulator))
  (let ((objects (list ;(find-instance-by-name 'self 'physical-object)
                  (find-instance-by-name 'robot 'robot)
                  (find-instance-by-name 'front-box 'physical-object))))
    (make-instance 'simulator 
                   :instance-name 'push
                   :objects objects
                   :policy-map (let ((ht (make-hash-table :test 'eq)))
                                 (setf (gethash (first objects) ht) 'move-forward-policy)
                                 ht)))))

(defun push-sphere-sim ()
  (if (null (find-instance-by-name 'push-sphere 'simulator))
  (let ((objects (list ;(find-instance-by-name 'self 'physical-object)
                  (find-instance-by-name 'robot 'robot)
                  (find-instance-by-name 'sphere 'physical-object))))
    (make-instance 'simulator 
                   :instance-name 'push-sphere
                   :objects objects
                   :policy-map (let ((ht (make-hash-table :test 'eq)))
                                 (setf (gethash (first objects) ht) 'move-forward-policy)
                                 ht)))))

(defun carry-sim ()
  (if (null (find-instance-by-name 'carry 'simulator))
  (let ((objects (list ;(find-instance-by-name 'self 'physical-object) 
                  (find-instance-by-name 'robot 'robot)
                  (find-instance-by-name 'above-box 'physical-object))))
    (make-instance 'simulator 
                   :instance-name 'carry
                   :objects objects
                   :policy-map (let ((ht (make-hash-table :test 'eq)))
                                 (setf (gethash (first objects) ht) 'move-forward-policy)
                                 ht)))))

(defun carry-sphere-sim ()
  (if (null (find-instance-by-name 'carry-sphere 'simulator))
      (let ((objects (list ;(find-instance-by-name 'self 'physical-object) 
                      (find-instance-by-name 'robot 'robot)
                      (find-instance-by-name 'above-sphere 'physical-object))))
        (make-instance 'simulator 
                       :instance-name 'carry-sphere
                       :objects objects
                       :policy-map (let ((ht (make-hash-table :test 'eq)))
                                     (setf (gethash (first objects) ht) 'move-forward-policy)
                                     ht)))))

(defun block-sim ()
  (if (null (find-instance-by-name 'block 'simulator))
      (let ((objects (list (find-instance-by-name 'robot 'robot)
                                        ;(find-instance-by-name 'self 'physical-object) 
                           (find-instance-by-name 'stuck-box 'physical-object))))
        (make-instance 'simulator 
                       :instance-name 'block
                       :objects objects
                       :policy-map (let ((ht (make-hash-table :test 'eq)))
                                     (setf (gethash (first objects) ht) 'move-forward-policy)
                                     ht)))))

(defun double-block-sim ()
  (if (null (find-instance-by-name 'double-block 'simulator))
      (let ((objects (list (find-instance-by-name 'robot 'robot)
                           (find-instance-by-name 'stuck-box 'physical-object)
                           (find-instance-by-name 'front-box 'physical-object))))
        (make-instance 'simulator 
                       :instance-name 'double-block
                       :objects objects
                       :policy-map (let ((ht (make-hash-table :test 'eq)))
                                     (setf (gethash (first objects) ht) 'move-forward-policy)
                                     ht)))))

(defun hill-sim ()
  (if (null (find-instance-by-name 'hill 'simulator))
      (let ((objects (list (find-instance-by-name 'robot 'robot)
                           (find-instance-by-name 'sphere 'physical-object)
                           (find-instance-by-name 'ramp 'physical-object))))
        (make-instance 'simulator 
                       :instance-name 'hill
                       :objects objects
                       :policy-map (let ((ht (make-hash-table :test 'eq)))
                                     (setf (gethash (first objects) ht) 'demo-policy)
                                     ht)
                       :termination-time 30))))

;;=====================================================

(defun init-simulators ()
  (make-simulator-space)
  (free-sim)
  (push-sim)
  (carry-sim)
  (block-sim)
  (double-block-sim)
  (push-sphere-sim)
  (carry-sphere-sim)
  (hill-sim))

;;=====================================================

(defun cleanup ()
  (shutdown-ros-node)
  (delete-blackboard-repository))

(defun get-ready ()
  ;(init-objects)
  ;(init-simulators)
  (if (empty-blackboard-repository-p)
      (restore))
  (subscribe-to-world-state))

(defun uphill ()
  (cleanup)
  (with-ros-node ("simulators")
    (get-ready)
    (test-simulator (find-instance-by-name 'hill 'simulator))))

(defun test-all-simulators ()
  (with-ros-node ("simulators")
    (get-ready)

    (test-simulator (find-instance-by-name 'carry-sphere 'simulator))
    (test-simulator (find-instance-by-name 'push-sphere 'simulator))

    (test-simulator (find-instance-by-name 'free 'simulator))
    (test-simulator (find-instance-by-name 'push 'simulator))
    (test-simulator (find-instance-by-name 'carry 'simulator))
    (test-simulator (find-instance-by-name 'block 'simulator))))

(defun stress-test ()
  (dotimes (x 20)
    (test-all-simulators)))

(defun easy-test (sim)
  (with-ros-node ("simtest")
    (get-ready)
    (test-simulator (find-instance-by-name sim 'simulator))))

(defun test-blocking (sim obj)
  (with-ros-node ("simtest")
    (get-ready)
    (if (is-blocking? sim obj)
        (format t "~%~%~%Answer: YES, ~a is blocking the robot." obj)
        (format t "~%~%~%Answer: NO, ~a is not blocking the robot." obj))))
    
(defun is-blocking? (sim-name obj-name)
  (let* ((sim (find-instance-by-name sim-name 'simulator))
         (obj (find-instance-by-name obj-name)))
    ;; First, simulate to see if there is any blockage at all
    (run-simulator sim)
    ;; If there is, we can test to see if the object was responsible (i.e., "caused" it)
    (if (not (success-of (current-simulation sim)))
        (let* ()
          (suppress sim obj) ;; Intervention 
          (run-simulator sim) ;; Simple counterfactual test
          (revive sim obj) ;; Restore the simulator for future runs
          (success-of (current-simulation sim))))))

(defun test-simulator (sim)
    (run-simulator sim)
    (sleep 2))
    

;;=====================================================
;; Policy

(defun move-forward-policy (obj sim-time ws)
  (declare (ignore ws)
           (ignore sim-time))
  (if (eq 'robot (class-name (class-of obj)))
      (move-robot obj 0.5 0.0)
      (apply-force obj '(200 0 0) 0.1)))
          
(defun seek-goal-policy (obj sim-time ws)
  (declare (ignore sim-time))
  (if ws
      (if (eq 'robot (class-name (class-of obj)))
          (let* ((r-pos (position-of (pose-of (get-object-state ws obj))))
                 (goal '(6.0 0.0)))
            (if (> (x-of r-pos) (first goal))
                (move-robot obj 0.0 0.0)
                (move-robot obj 0.5 (point-at-natural (x-of r-pos) (y-of r-pos)
                                                      (first goal) (second goal)))))
          (apply-force obj '(200 0 0) 0.1))))
  
;;=======================================================

(defun clear-simulations ()
  (loop for sim in (find-instances-of-class 'simulator)
     for si = (find-space-instance-by-path (list (instance-name-of sim)))
     when si do (delete-space-instance si)))

;;=======================================================

(defun test-script ()
  (with-ros-node ("simulators")
    ;(get-ready)
    (subscribe-to-world-state)

    (lower-robot)
    (test-simulator (find-instance-by-name 'block 'simulator))
    (test-simulator (find-instance-by-name 'free 'simulator))
    
    (raise-robot)
    (open-gap)
    (test-simulator (find-instance-by-name 'gap 'simulator))
    (close-gap)
    (test-simulator (find-instance-by-name 'gap 'simulator))))