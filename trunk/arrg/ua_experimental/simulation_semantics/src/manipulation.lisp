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
  (call-service "gazebo/set_model_state" 'gazebo-srv:SetModelState
                      :model_state (make-msg "gazebo/ModelState"
                                             (model_name) (model-name object)
                                             (x position pose) (if x x (first (xyz-of object)))
                                             (y position pose) (if y y (second (xyz-of object)))
                                             (z position pose) (if z z (third (xyz-of object))))))

(defun resize-object (object &key (scale nil) (x nil) (y nil) (z nil))
  (remove-from-world object)
  (cond (scale (setf (size-of object) scale)
               (setf (third (xyz-of object)) (size-of object))
               (make-xml-list object)))
  (add-to-world object))
  