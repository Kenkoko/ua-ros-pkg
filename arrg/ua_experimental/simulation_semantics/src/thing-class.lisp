(in-package :simulation_semantics)

;;====================================================
;; Space where objects live (needed to index them by name)

(defun add-to-object-space (entity)
  (declare (ignore entity))
  (make-object-space))

(defun make-object-space ()
  (if (null (find-space-instance-by-path '(object-library)))
      (make-space-instance '(object-library)
                           :dimensions (dimensions-of 'physical-object))
      (find-space-instance-by-path '(object-library))))

;;====================================================
;; Class Definition 

(define-unit-class entity ()
  (gazebo-name ;; no initform so it must be bound
   (default-xyz :initform (list 0 0 0))
   (self-predicates :initform :all)
   (binary-predicates :initform (list 'dist-between 'rel-angle))
   (xml-string :initform nil))
  (:initial-space-instances 
   #'add-to-object-space) ;; This is handy
  (:dimensional-values
   (gazebo-name :element gazebo-name)))

(defgeneric activate (entity)
  (:documentation "Create the publishers necessary to control the object."))

(defgeneric deactivate (entity)
  (:documentation "Take down the control publishers."))

(defgeneric add-to-world (entity pose)
  (:documentation "Spawn the object in Gazebo."))

(defgeneric remove-from-world (entity)
  (:documentation "Remove the object from Gazebo."))

(defgeneric xml-rep (entity)
  (:documentation "Generate an XML representation for this entity."))

#+ignore(defgeneric get-default-predicates (entity)
  (:documentation "Returns the appropriate default set of predicates for the entity type."))

(defun make-initial-pose (pose)
  (let* ((xyz (first pose))
         (rpy (second pose))
         (quat (cl-transforms:euler->quaternion :ax (first rpy)
                                                :ay (second rpy)
                                                :az (third rpy))))
    (make-message "geometry_msgs/Pose"
                  :position (make-message "geometry_msgs/Point"
                                          :x (first xyz)
                                          :y (second xyz)
                                          :z (third xyz))
                  :orientation (make-message "geometry_msgs/Quaternion"
                                             :x (cl-transforms:x quat)
                                             :y (cl-transforms:y quat)
                                             :z (cl-transforms:z quat)
                                             :w (cl-transforms:w quat)))))
