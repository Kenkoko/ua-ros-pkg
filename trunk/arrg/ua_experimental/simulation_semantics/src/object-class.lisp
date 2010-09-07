(in-package :simulation_semantics)

;;====================================================
;; Class Definition 

(define-unit-class physical-object (entity)
  ((shape :initform 'box)
   (color :initform "Gazebo/Blue")
   (size :initform 0.2)
   (mass :initform 0.2)
   (static? :initform nil)) 
  (:dimensional-values 
   (static? :boolean static?))
)

;;====================================================
;; Constructor

;(defmethod initialize-instance :after ((obj physical-object) &key)
;  (make-xml-list obj))

(defun make-xml-list (obj pose)
  (setf (xml-string-of obj) 
        (let ((xml-list (list (gen-header obj)
                              (xyz-xml (first pose))
                              (rpy-xml (second pose))
                              (list :|static| (boolean-string (static?-of obj)))
                              (body-xml obj))))
          (concatenate 'string "<?xml version=\"1.0\"?>"
                       (print-xml-string xml-list :pretty t)))))

;;====================================================
;; ROS-based methods

(defmethod activate ((obj physical-object))
  "Creates a ROS publisher for the object's relevant topics (does nothing for now)")

(defmethod deactivate ((obj physical-object))
  "Un-registers ROS publisher for the object's relevant topics (does nothing for now)")

(defmethod add-to-world ((obj physical-object) pose)
  (call-service "gazebo/spawn_gazebo_model" 'gazebo-srv:SpawnModel 
                :model_name (model-name obj)
                :model_xml (make-xml-list obj pose)
                :initial_pose (make-initial-pose pose)
                :robot_namespace "/"
))                

(defmethod remove-from-world ((obj physical-object))
  (call-service "gazebo/delete_model" 'gazebo-srv:DeleteModel
                :model_name (model-name obj)))

(defun height-of (obj)
  (let* ((size (size-of obj)))
    (if (numberp size)
        size
        (third size))))

(defmethod get-default-predicates ((obj physical-object))
  '(x-pos y-pos z-pos
    ;x-vel y-vel z-vel 
    vel-mag ;diff-speed
    ;force-mag
    yaw pitch roll))
    ;yaw-vel pitch-vel roll-vel))

;;======================================================
;; Manipulating objects

(defmethod move-to ((obj physical-object) new-pos)
  (call-service "gazebo/set_model_state" 'gazebo-srv:SetModelState
                :model_state (make-msg "gazebo/ModelState"
                                       (model_name) (model-name obj)
                                       (x position pose) (first new-pos)
                                       (y position pose) (second new-pos)
                                       (z position pose) (third new-pos))))
                                           
(defmethod apply-force ((obj physical-object) force duration)
  (call-service "gazebo/apply_body_wrench" 'gazebo-srv:ApplyBodyWrench
                :body_name (concatenate 'string (model-name obj) "::" (gazebo-name-of obj))
                :wrench (make-msg "geometry_msgs/Wrench"
                                  (x force) (first force)
                                  (y force) (second force)
                                  (z force) (third force))
                :start_time (ros-time)
                :duration duration
))



;;======================================================
;; Gazebo XML Generation

;(defmethod xml-rep ((obj physical-object)) 
;  (xml-string-of obj))

(defmethod gen-header ((obj physical-object))
  (register-namespace "http://playerstage.sourceforge.net/gazebo/xmlschema/#model" "model" "model")
  (list '|model|:|physical| 
        ':|xmlns:gazebo| "http://playerstage.sourceforge.net/gazebo/xmlschema/#gz" 
        ':|xmlns:model| "http://playerstage.sourceforge.net/gazebo/xmlschema/#model" 
        ':|xmlns:sensor| "http://playerstage.sourceforge.net/gazebo/xmlschema/#sensor" 
        ':|xmlns:body| "http://playerstage.sourceforge.net/gazebo/xmlschema/#body"
        ':|xmlns:geom| "http://playerstage.sourceforge.net/gazebo/xmlschema/#geom"
        ':|xmlns:joint| "http://playerstage.sourceforge.net/gazebo/xmlschema/#joint"
        ':|xmlns:controller| "http://playerstage.sourceforge.net/gazebo/xmlschema/#controller"
        ':|xmlns:interface| "http://playerstage.sourceforge.net/gazebo/xmlschema/#interface"
        ':|xmlns:rendering| "http://playerstage.sourceforge.net/gazebo/xmlschema/#rendering"
        ':|xmlns:physics| "http://playerstage.sourceforge.net/gazebo/xmlschema/#physics" 
        ':|name| (model-name obj)))

(defmethod model-name ((obj physical-object))
  ;(concatenate 'string (gazebo-name-of obj) "_model"))
  (gazebo-name-of obj))

;(defmethod set-xyz ((obj physical-object) x y z)
;  (setf (xyz-of obj) (list x y z)))

(defun xyz-xml (xyz)
  (list :|xyz| (format nil "~{~a ~}" xyz))) 

(defun rpy-xml (rpy)
  (list :|rpy| (format nil "~{~a ~}" rpy)))

(defmethod get-shape-xml ((obj physical-object))
  (cond ((eq (shape-of obj) 'sphere) 
         '|body|:|sphere|)
        (t 
         '|body|:|box|)))

(defmethod get-size-vector ((obj physical-object))
  (if (numberp (size-of obj))
      (list (size-of obj) (size-of obj) (size-of obj))
      (size-of obj)))

(defmethod get-size-xml ((obj physical-object))
  (cond ((eq (shape-of obj) 'sphere) 
         (format nil "~d" (size-of obj)))
        (t 
         (format nil "~{~a ~}" (get-size-vector obj)))))

(defmethod get-mesh-xml ((obj physical-object))
  (cond ((eq (shape-of obj) 'sphere) "unit_sphere")
        (t "unit_box")))

(defmethod get-scale-xml ((obj physical-object))
  (format nil "~{~a ~}" (get-size-vector obj)))

(defmethod get-geom-xml ((obj physical-object))
  (cond ((eq (shape-of obj) 'sphere) '|geom|:|sphere|)
        (t '|geom|:|box|)))

(defmethod body-xml ((obj physical-object))
  (append (list (list (get-shape-xml obj) :|name| (gazebo-name-of obj)))
          (append `((:|turnGravityOff| "false")
                    (:|dampingFactor| "0.01") 
                    (:|selfCollide| "false") 
                    (:|massMatrix| "true")
                    (:|mass| ,(format nil "~d" (mass-of obj))) 
                    (:|ixx| "0.1") (:|ixy| "0.0") (:|ixz| "0.0")
                    (:|iyy| "0.1") (:|iyz| "0.0") 
                    (:|izz| "0.1") 
                    (:|cx| "0.0") (:|cy| "0.0") (:|cz| "0.0") 
                    (:|xyz| "0 0 0") (:|rpy| "0 0 0")) ;; WHA??,(rpy-xml obj))
                  (list (append (list (list (get-geom-xml obj) 
                                            :|name| (concatenate 'string 
                                                                  (gazebo-name-of obj)
                                                                  "_geom")))
                                (list '(:|mu1| "1.0") 
                                      '(:|mu2| "1.0")
                                      '(:|kp| "10000.0") 
                                      '(:|kd| "1.0") 
                                      (list :|size| (get-size-xml obj))
                                      (list :|mass| (format nil "~d" (mass-of obj)))
                                      (list :|visual| 
                                            '(:|xyz| "0 0 0") 
                                            '(:|rpy| "0 0 0") 
                                            (list :|scale| (get-scale-xml obj))
                                            (list :|mesh| (get-mesh-xml obj)) 
                                            (list :|material| (color-of obj)))))))))

;;=====================================================
;; Helpers

(defun boolean-string (value)
  (if value "true" "false"))
