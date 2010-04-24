(in-package :simulation_semantics)

;; TODO: Test all this stuff

(define-space-class simulation ()
  ((simulator 
    :link (simulator simulations)
    :singular t)))

(defmethod create-simulation ((sim simulator))
  (let* ((simulation-count (length (simulations-of sim))))
    (if (= 0 simulation-count)
        (make-space-instance (list (instance-name-of sim))))
    (linkf (simulations-of sim) 
           (make-space-instance (list (instance-name-of sim)
                                      (gensym "SIMULATION"))
                                :class 'simulation
                                :allowed-unit-classes '(world-state)
                                :dimensions (dimensions-of 'world-state)))))

(define-unit-class world-state ()
  (time
   (objects
    :link (object-state world :singular t)))
  (:dimensional-values
   (time :point time)))

(define-unit-class object-state ()
  ((world
    :link (world-state objects)
    :singular t)
   object
   pose
   force
   velocity))

(define-unit-class xyz ()
  ((x :initform 0.0)
   (y :initform 0.0)
   (z :initform 0.0))
  (:dimensional-values
   (x :point x)
   (y :point y)
   (z :point z)))

(define-unit-class quaternion ()
  ((x :initform 0.0)
   (y :initform 0.0)
   (z :initform 0.0)
   (w :initform 0.0))
  (:dimensional-values
   (x :point x)
   (y :point y)
   (z :point z)
   (w :point w)))

(define-unit-class pose ()
  (position
   orientation))
   
(define-unit-class force ()
  (linear
   torque))

(define-unit-class velocity ()
  (linear
   angular))