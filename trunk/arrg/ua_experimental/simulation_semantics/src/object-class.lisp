(in-package :simulation_semantics)

;;====================================================
;; Class Definition 

(define-unit-class physical-object ()
  ((gazebo-name :initform "blue_box")
   (shape :initform 'box)
   (color :initform "Gazebo/Blue")
   (xyz  :initform '(0 0 0))
   (static? :initform nil)
   
   ;; "Private" fields
   (xml-string :initform nil)
   (force-pub :initform nil))
  (:initial-space-instances (object-library))
)

;;====================================================
;; Constructor

(defmethod initialize-instance :after ((obj physical-object) &key)
  (setf (xml-string-of obj) 
        (let ((xml-list (list (gen-header obj)
                              (xyz-xml obj)
                              (list :|rpy| "0 0 0")
                              (list :|static| (boolean-string (static?-of obj)))
                              (body-xml obj))))
          (if (not (static?-of obj))
              (setf xml-list (append xml-list (list (force-xml obj)))))
          (print-xml-string xml-list :pretty nil))))

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

(defmethod body-xml ((obj physical-object))
  (cond ((eq (shape-of obj) 'box)
         (append (list (list '|body|:|box| 
                             ':|name| (concatenate 'string (gazebo-name-of obj) "_body"))) 
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
                         (list (append (list (list '|geom|:|box| 
                                                   ':|name| (concatenate 'string 
                                                                         (gazebo-name-of obj)
                                                                         "_geom")))
                                       (list '(:|mu1| "1.0") 
                                             '(:|mu2| "1.0")
                                             '(:|kp| "100000000.0") 
                                             '(:|kd| "1.0") 
                                             '(:|size| "0.2 0.2 0.2") 
                                             '(:|mass| "0.2")
                                             (list :|visual| 
                                                   '(:|xyz| "0 0 0") 
                                                   '(:|rpy| "0 0 0") 
                                                   '(:|scale| "0.2 0.2 0.2")
                                                   '(:|mesh| "unit_box") 
                                                   (list :|material| (color-of obj))))))))
          )
        (t (format t "DEATH DEATH DEATH~%"))))

(defmethod force-xml ((obj physical-object))
  (let* ((name (gazebo-name-of obj))
         (body (concatenate 'string name "_body"))
         (controller (concatenate 'string name "_force"))
         (iface (concatenate 'string controller "_iface")))
  (list (list '|controller|:|gazebo_ros_force| 
              ':|name| controller
              ':|plugin| "libgazebo_ros_force.so")
        '(:|alwaysOn| "true") 
        '(:|updateRate| "15.0") 
        (list ':|topicName| controller)
        (list ':|bodyName| body) 
        '(:|robotNamespace| "/")
        (list (list '|interface|:|position| ':|name| iface)))))

;;=====================================================
;; Helpers

(defun boolean-string (value)
  (if value "true" "false"))
