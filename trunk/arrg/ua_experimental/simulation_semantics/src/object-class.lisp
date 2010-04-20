(in-package :simulation_semantics)

;; TODO: Un-hardcode this, we are only doing this avoid symbol package problems
;; created by s-xml
;; ... and it doesn't seem to do that anyway
(defparameter *example-xml* 
  (parse-xml-file "/home/dhewlett/ros/ua-ros-pkg/arrg/ua_experimental/simulator_experiments/objects/blue_box.xml"))

;;====================================================
;; Class Definition 

(defclass physical-object ()
  ((gazebo-name :accessor gazebo-name :initarg :gazebo-name :initform "blue_box")
   (shape :accessor shape :initform 'box)
   (xyz :accessor xyz :initarg :xyz :initform '(0 0 0))
   (is-static :accessor is-static :initarg :is-static :initform nil)
   
   ;; "Private" fields
   (xml-string :accessor xml-string :initform nil)
   (force-pub :accessor force-pub :initform nil)
 )
)

;;====================================================
;; Constructor

(defmethod initialize-instance :after ((obj physical-object) &key)
  (setf (xml-string obj) 
        (let ((xml-list (list (gen-header obj)
                              (xyz-xml obj)
                              (list :|rpy| "0 0 0")
                              (list :|static| (boolean-string (is-static obj)))
                              (body-xml obj))))
          (if (not (is-static obj))
              (setf xml-list (append xml-list (list (force-xml obj)))))
          (print-xml-string xml-list :pretty nil)))
  (setf (force-pub obj) (advertise (concatenate 'string (gazebo-name obj) "_force") 
                                   "geometry_msgs/Wrench")))

;;====================================================
;; ROS-based methods

(defmethod add-to-world ((obj physical-object))
  (call-service "add_model" 'gazebo_plugins-srv:SpawnModel 
                :model (make-message "gazebo_plugins/GazeboModel" 
                                     :xml_type 1
                                     :robot_model (xml-rep obj))))

(defmethod remove-from-world ((obj physical-object))
  (call-service "delete_model" 'gazebo_plugins-srv:DeleteModel
                :model_name (model-name obj)))

(defmethod apply-force ((obj physical-object) force)
  (publish-msg (force-pub obj) 
               (x force) (first force)
               (y force) (second force)
               (z force) (third force))
)



;;======================================================
;; Gazebo XML Generation

(defmethod xml-rep ((obj physical-object)) 
  (xml-string obj))

(defmethod gen-header ((obj physical-object))
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
  (concatenate 'string (gazebo-name obj) "_model"))

(defmethod set-xyz ((obj physical-object) x y z)
  (setf (xyz obj) (list x y z)))

(defmethod xyz-xml ((obj physical-object))
  (list :|xyz| (format nil "~{~a ~}" (xyz obj)))) 

(defmethod body-xml ((obj physical-object))
  (cond ((eq (shape obj) 'box)
         (append (list (list '|body|:|box| 
                             ':|name| (concatenate 'string (gazebo-name obj) "_body"))) 
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
                                                                         (gazebo-name obj)
                                                                         "_geom")))
                                       '((:|mu1| "1.0") 
                                         (:|mu2| "1.0")
                                         (:|kp| "100000000.0") 
                                         (:|kd| "1.0") 
                                         (:|size| "0.2 0.2 0.2") 
                                         (:|mass| "0.2")
                                         (:|visual| 
                                          (:|xyz| "0 0 0") 
                                          (:|rpy| "0 0 0") 
                                          (:|scale| "0.2 0.2 0.2")
                                          (:|mesh| "unit_box") 
                                          (:|material| "Gazebo/Blue")))))))
          )
        (t (format t "DEATH DEATH DEATH~%"))))

(defmethod force-xml ((obj physical-object))
  (let* ((name (gazebo-name obj))
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
