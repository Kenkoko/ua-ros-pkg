(in-package :simulation_semantics)

;;====================================================
;; Class Definition 

(define-unit-class thing ()
  (gazebo-name ;; no initform so it must be bound
   (xyz  :initform (list 1.0 0 0.2))
   (rpy :initform (list 0 0 0))
   (default-xyz :initform (list 0 0 0))
   (self-predicates :initform (list 'diff-speed 
                                    'x-pos 'y-pos 'z-pos 
                                    'x-vel 'y-vel 'z-vel 
                                    'force-mag))
   (binary-predicates :initform (list 'dist-between))
   (xml-string :initform nil))
  (:initial-space-instances 
   (object-library))
  (:dimensional-values
   (gazebo-name :element gazebo-name)))

(defgeneric activate (entity)
  (:documentation "Create the publishers necessary to control the object."))

(defgeneric deactivate (entity)
  (:documentation "Take down the control publishers."))

(defgeneric add-to-world (entity)
  (:documentation "Spawn the object in Gazebo."))

(defgeneric remove-from-world (entity)
  (:documentation "Remove the object from Gazebo."))

(defgeneric xml-rep (entity)
  (:documentation "Generate an XML representation for this entity."))

(defgeneric get-initial-pose (entity)
  (:documentation "Get a pose object for sending to Gazebo."))
