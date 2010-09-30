(in-package :simulation_semantics)

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
   (predicates :initform nil)
   (simulation :initform nil))
  (:dimensional-values
   (time :point time)))

;(defmethod initialize-instance :after ((ws world-state) &key)
;  (annotate-with-predicates ws))

(defmethod get-object-state ((ws world-state) (obj entity))
  (loop with result = nil
     for candidate in (objects-of ws)
     when (eq (object-of candidate) obj)
     do (setf result candidate)
     finally (return result)))

;(defmethod simulation-of ((ws world-state))
;  (first (space-instances-of ws)))

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

(defun translate-pose (msg)
  (list (translate-xyz (geometry_msgs-msg:position-val msg))
        (translate-xyzw (geometry_msgs-msg:orientation-val msg))))

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

;; http://sunday-lab.blogspot.com/2008/04/get-pitch-yaw-roll-from-quaternion.html

(defmethod get-yaw ((this quaternion))
  (atan (* 2 (+ (* (x-of this) (y-of this))
                (* (w-of this) (z-of this))))
        (+ (expt (w-of this) 2)
           (expt (x-of this) 2)
           (- (expt (y-of this) 2))
           (- (expt (z-of this) 2)))))

(defmethod get-pitch ((this quaternion))
  (atan (* 2 (+ (* (y-of this) (z-of this))
                (* (w-of this) (x-of this))))
        (+ (expt (w-of this) 2)
           (- (expt (x-of this) 2))
           (- (expt (y-of this) 2))
           (expt (z-of this) 2))))

(defmethod get-roll ((this quaternion))
  (asin (* -2 (- (* (x-of this) (z-of this))
                 (* (w-of this) (y-of this))))))

;; Convert (-pi,pi) to (0,2*pi)
(defun shift-angle (theta)
  (if (< theta 0)
      (+ theta (* 2 pi))
      (mod theta (* 2 pi))))

;; Convert (0,2*pi) to (-pi, pi)
(defun unshift-angle (theta)
  (if (< theta 0)
      (print "WTF MATE"))
  (if (> theta pi)
      (- theta (* 2 pi))
      theta))

(defmethod get-abs-yaw ((this quaternion))
  (shift-angle (get-yaw this)))

;; Returns a shifted angle
(defun point-at-angle (my-x my-y other-x other-y)
  (shift-angle (atan (- other-y my-y) (- other-x my-x))))

(defun point-at-natural (my-x my-y other-x other-y)
  (unshift-angle (shift-angle (atan (- other-y my-y) (- other-x my-x)))))

;; left-of is positive, right-of is negative
;; Expects an abs-yaw
(defun relative-angle (my-yaw my-x my-y other-x other-y)
  (let* ((angle (- (point-at-angle my-x my-y other-x other-y)
                   my-yaw)))
    (unshift-angle (if (< angle 0)
                       (+ (* 2 pi) angle)
                       angle))))

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

