(in-package :simulation_semantics)

;;===========================================================

(define-space-class simulation ()
  ((simulator 
    :link (simulator simulations)
    :singular t)))

;;===========================================================
;; Extra methods for simulator class, probably want to move these

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

(defmethod current-simulation ((sim simulator))
  (let* ((simulations (simulations-of sim)))
    (if (null simulations)
        nil
        (first simulations))))


;;===========================================================

(define-unit-class world-state ()
  (time
   (objects
    :link (object-state world :singular t))
   (prev-state 
    :link (world-state next-state :singular t)
    :singular t)
   (next-state
    :link (world-state prev-state :singular t)
    :singular t)
   predicates)
  (:dimensional-values
   (time :point time)))

(defmethod initialize-instance :after ((ws world-state) &key)
  (annotate-with-predicates ws))


;;===========================================================

(define-unit-class object-state ()
  ((world
    :link (world-state objects)
    :singular t)
   object
   pose
   force
   velocity))

;;===========================================================

(define-unit-class xyz ()
  ((x :initform 0.0)
   (y :initform 0.0)
   (z :initform 0.0))
  (:dimensional-values
   (x :point x)
   (y :point y)
   (z :point z)))

(defmethod print-instance-slots ((this xyz) stream)
  (call-next-method)
  (default-print-slots this stream))

(defmethod distance ((p1 xyz) (p2 xyz))
  (sqrt (+ (expt (- (x-of p1) (x-of p2)) 2)
           (expt (- (y-of p1) (y-of p2)) 2)
           (expt (- (z-of p1) (z-of p2)) 2))))

(defun translate-xyz (msg)
  (make-instance 'xyz 
                 :x (geometry_msgs-msg:x-val msg)
                 :y (geometry_msgs-msg:y-val msg)
                 :z (geometry_msgs-msg:z-val msg)))

(defmethod as-list ((this xyz))
  (list (x-of this) (y-of this) (z-of this)))

;;===========================================================

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

(defmethod print-instance-slots ((this quaternion) stream)
  (call-next-method)
  (default-print-slots this stream))

(defun translate-xyzw (msg)
  (make-instance 'quaternion 
                 :x (geometry_msgs-msg:x-val msg)
                 :y (geometry_msgs-msg:y-val msg)
                 :z (geometry_msgs-msg:z-val msg)
                 :w (geometry_msgs-msg:w-val msg)))

;;===========================================================

(define-unit-class pose ()
  (position
   orientation))
   
(defmethod print-instance-slots ((this quaternion) stream)
  (call-next-method)
  (default-print-slots this stream))

;;===========================================================

(define-unit-class force ()
  (linear
   torque))

(defmethod print-instance-slots ((this force) stream)
  (call-next-method)
  (default-print-slots this stream))

;;===========================================================

(define-unit-class velocity ()
  (linear
   angular))

(defmethod print-instance-slots ((this velocity) stream)
  (call-next-method)
  (default-print-slots this stream))

;;===========================================================
;; Utility method for printing simple objects

(defun default-print-slots (obj stream)
  (format stream " ~{~a~^, ~}"
          (loop with obj-class = (class-of obj)
             for slot-def in (class-direct-slots obj-class)
             for slot-name = (slot-definition-name slot-def)
             collect (format nil "~a: ~a" slot-name
                             (if (slot-boundp obj slot-name)
                                 (slot-value obj slot-name) 
                                 "<UNBOUND>")))))