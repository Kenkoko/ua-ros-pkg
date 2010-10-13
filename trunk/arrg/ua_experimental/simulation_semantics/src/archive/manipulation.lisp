(in-package :simsem)

;; TODO: All of these are out of date

#+ignore(defun move-object (object &key (x nil) (y nil) (z nil))
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

#+ignore(defun resize-object (object &key (scale nil) (size nil))
  (remove-from-world object)
  (cond (scale (setf (size-of object) scale)
               (setf (third (xyz-of object)) (size-of object))
               (make-xml-list object))
        (size (setf (size-of object) size)
              (setf (third (xyz-of object)) (third (size-of object)))
              (make-xml-list object)))
  (add-to-world object))

#+ignore(defun rotate-object (object rpy)
  (remove-from-world object)
  (setf (rpy-of object) rpy)
  (make-xml-list object)
  (add-to-world object))

#+ignore(defun morph-object (object new-shape)
  (remove-from-world object)
  (setf (shape-of object) new-shape)
  (make-xml-list object)
  (add-to-world object))

#+ignore(defun change-mass (object new-mass)
  (remove-from-world object)
  (setf (mass-of object) new-mass)
  (make-xml-list object)
  (add-to-world object))

;(defun raise-robot ()
;  (let ((robot (find-instance-by-name 'robot)))
;    (setf (third (xyz-of robot)) 0.6)))

;(defun lower-robot ()
;  (let ((robot (find-instance-by-name 'robot)))
;    (setf (third (xyz-of robot)) 0.1)))

#+ignore(defun close-gap ()
  (let ((floor (find-instance-by-name 'floor2)))
    (setf (first (xyz-of floor)) 5.0)
    (make-xml-list floor)))

#+ignore(defun open-gap ()
  (let ((floor (find-instance-by-name 'floor2)))
    (setf (first (xyz-of floor)) 6.0)
    (make-xml-list floor)))
          
