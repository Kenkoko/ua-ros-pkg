(in-package :simsem)

;;===========================================================

(define-unit-class manipulable ()
  ((manipulators :initform nil))
)

(define-unit-class manipulator ()
  (name
   (value-type :initform 'discrete)
   (value-range :initform '(on off))
   distribution))

;;===========================================================

(defgeneric get-manipulators (entity)
  (:documentation "Returns the set of manipulations that can be performed on a given entity."))

;(defmethod get-manipulators ((object physical-object))
;  '(position rotation 

;;============================================================

(defun move-object (object &key (x nil) (y nil) (z nil))
  (let ((pos (xyz-of object)))
    (if x (setf (first pos) x))
    (if y (setf (second pos) y))
    (if z (setf (third pos) z))
    (call-service "gazebo/set_model_state" 'gazebo-srv:SetModelState
                  :model_state (make-msg "gazebo/ModelState"
                                         (model_name) (model-name object)
                                         (x position pose) (first pos)
                                         (y position pose) (second pos)
                                         (z position pose) (third pos)
                                         (w orientation pose) 1.0))))

(defun resize-object (object &key (scale nil) (size nil))
  (remove-from-world object)
  (cond (scale (setf (size-of object) scale)
               (setf (third (xyz-of object)) (size-of object))
               (make-xml-list object))
        (size (setf (size-of object) size)
              (setf (third (xyz-of object)) (third (size-of object)))
              (make-xml-list object)))
  (add-to-world object))

(defun raise-robot ()
  (let ((robot (find-instance-by-name 'robot)))
    (setf (third (xyz-of robot)) 0.6)))

(defun lower-robot ()
  (let ((robot (find-instance-by-name 'robot)))
    (setf (third (xyz-of robot)) 0.1)))

(defun close-gap ()
  (let ((floor (find-instance-by-name 'floor2)))
    (setf (first (xyz-of floor)) 5.0)
    (make-xml-list floor)))

(defun open-gap ()
  (let ((floor (find-instance-by-name 'floor2)))
    (setf (first (xyz-of floor)) 6.0)
    (make-xml-list floor)))
          
