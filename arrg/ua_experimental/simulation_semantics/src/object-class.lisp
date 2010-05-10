(in-package :simulation_semantics)

;;====================================================
;; Class Definition 

(define-unit-class physical-object ()
  ((gazebo-name :initform "blue_box") ;; This name is used for the body
   (shape :initform 'box)
   (color :initform "Gazebo/Blue")
   (size :initform 0.2)
   (mass :initform 0.2)
   (xyz  :initform '(0 0 0))
   (rpy :initform '(0 0 0))
   (static? :initform nil)
   ;; Predicates
   (self-predicates :initform nil)
   (binary-predicates :initform nil)
   ;; "Private" fields
   (xml-string :initform nil)
   (force-pub :initform nil))
  (:initial-space-instances (object-library))
  (:dimensional-values 
   (static? :boolean static?)
   (gazebo-name :element gazebo-name))
)

;;====================================================
;; Constructor

(defmethod initialize-instance :after ((obj physical-object) &key)
  (setf (xml-string-of obj) 
        (let ((xml-list (list (gen-header obj)
                              (xyz-xml obj)
                              (rpy-xml obj)
                              (list :|static| (boolean-string (static?-of obj)))
                              (body-xml obj))))
          (if (not (static?-of obj))
              (setf xml-list (append xml-list (list (force-xml obj)))))
          (concatenate 'string "<?xml version=\"1.0\"?>"
                       (print-xml-string xml-list :pretty nil)))))

;;====================================================
;; ROS-based methods

(defmethod activate ((obj physical-object))
  "Creates a ROS publisher for the object's relevant topics (just <name>_force for now)"
  (if (force-pub-of obj)
      (print "Warning: Object is already activated?")
      (setf (force-pub-of obj) (advertise (concatenate 'string (gazebo-name-of obj) "_force") 
                                          "geometry_msgs/Wrench"))))

(defmethod deactivate ((obj physical-object))
  "Un-registers ROS publisher for the object's relevant topics (just <name>_force for now)"
  (unadvertise (concatenate 'string (gazebo-name-of obj) "_force"))
  (setf (force-pub-of obj) nil))

(defmethod add-to-world ((obj physical-object))
  (call-service "add_model" 'gazebo_plugins-srv:SpawnModel 
                :model (make-message "gazebo_plugins/GazeboModel" 
                                     :xml_type 1
                                     :robot_model (xml-rep obj))))

(defmethod remove-from-world ((obj physical-object))
  (call-service "delete_model" 'gazebo_plugins-srv:DeleteModel
                :model_name (model-name obj)))

(defmethod apply-force ((obj physical-object) force)
  (publish-msg (force-pub-of obj) 
               (x force) (first force)
               (y force) (second force)
               (z force) (third force))
)



;;======================================================
;; Gazebo XML Generation

(defmethod xml-rep ((obj physical-object)) 
  (xml-string-of obj))

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
  (concatenate 'string (gazebo-name-of obj) "_model"))

(defmethod set-xyz ((obj physical-object) x y z)
  (setf (xyz-of obj) (list x y z)))

(defmethod xyz-xml ((obj physical-object))
  (list :|xyz| (format nil "~{~a ~}" (xyz-of obj)))) 

(defmethod rpy-xml ((obj physical-object))
  (list :|rpy| (format nil "~{~a ~}" (rpy-of obj))))

(defmethod get-shape-xml ((obj physical-object))
  (cond ((eq (shape-of obj) 'sphere) 
         '|body|:|sphere|)
        (t 
         '|body|:|box|)))

(defmethod get-size-vector ((obj physical-object))
  (if (numberp (size-of obj))
      (list (size-of obj) (size-of obj) (size-of obj))
      (list (first (size-of obj)) (second (size-of obj)) (third (size-of obj)))))

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
          (append '((:|turnGravityOff| "false")
                    (:|dampingFactor| "0.01") 
                    (:|selfCollide| "false") 
                    (:|massMatrix| "true")
                    (:|mass| "0.2") 
                    (:|ixx| "0.001") (:|ixy| "0.0") (:|ixz| "0.0")
                    (:|iyy| "0.001") (:|iyz| "0.0") 
                    (:|izz| "0.001") 
                    (:|cx| "0.0") (:|cy| "0.0") (:|cz| "0.0") 
                    (:|xyz| "0 0 0") (:|rpy| "0 0 0"))
                  (list (append (list (list (get-geom-xml obj) 
                                            :|name| (concatenate 'string 
                                                                  (gazebo-name-of obj)
                                                                  "_geom")))
                                (list '(:|mu1| "1.0") 
                                      '(:|mu2| "1.0")
                                      '(:|kp| "100000000.0") 
                                      '(:|kd| "1.0") 
                                      (list :|size| (get-size-xml obj))
                                      (list :|mass| (format nil "~d" (mass-of obj)))
                                      (list :|visual| 
                                            '(:|xyz| "0 0 0") 
                                            '(:|rpy| "0 0 0") 
                                            (list :|scale| (get-scale-xml obj))
                                            (list :|mesh| (get-mesh-xml obj)) 
                                            (list :|material| (color-of obj)))))))))

(defmethod force-xml ((obj physical-object))
  (let* ((name (gazebo-name-of obj))
         (controller (concatenate 'string name "_force"))
         (iface (concatenate 'string controller "_iface")))
  (list (list '|controller|:|gazebo_ros_force| 
              ':|name| controller
              ':|plugin| "libgazebo_ros_force.so")
        '(:|alwaysOn| "true") 
        '(:|updateRate| "15.0") 
        (list ':|topicName| controller)
        (list ':|bodyName| name) 
        '(:|robotNamespace| "/")
        (list (list '|interface|:|position| ':|name| iface)))))

;;=====================================================
;; Helpers

(defun boolean-string (value)
  (if value "true" "false"))
