(in-package :asdf)

(defsystem "simulation_semantics"
  :depends-on (roslisp
               cl-transforms
               std_srvs-srv
               std_msgs-msg
               gbbopen-ros
               ;; Messages and services
               ;simulation_semantics-srv
               oomdp_msgs-msg
               oomdp_msgs-srv
               verb_learning-msg
               verb_learning-srv
               time_series-msg
               time_series-srv
               gazebo-msg
               gazebo-srv
               gazebo_plugins-msg
               gazebo_plugins-srv
               geometry_msgs-msg
               wubble_description-srv
               move_base_msgs-msg
               plotter-srv)

   :components

   ;; Package definition and shared utilities
   ((:module "common"
             :components ((:file "package") ;; The package file
                          (:file "utils")
                          (:file "geometry-classes")
                          (:file "environment")
                          (:file "state")))

    ;; Entity classes for gazebo
    (:module "entity"
             :depends-on ("common")
             :components ((:file "xml") ;; TODO: Remove
                          (:file "entity-class" :depends-on ("xml"))
                          (:file "robot" :depends-on ("entity-class"))
                          (:file "object-class" :depends-on ("entity-class"))
                          (:file "goals" :depends-on ("entity-class"))))

    ;; Interactive teaching of the robot
    (:module "teaching"
             :depends-on ("common" "entity")
             :components ((:file "simulator-class")
                          (:file "verbs")
                          (:file "scenarios" :depends-on ("verbs"))
                          (:file "verb-teaching" :depends-on ("verbs"))))
    
    ;; Training and test data
    (:module "data"
             :depends-on ("common")
             :components ((:file "gazebo-training")
                          (:file "gazebo-test" :depends-on ("gazebo-training"))
                          (:file "ww2d-training")
                          (:file "ww2d-test" :depends-on ("ww2d-training"))))

    ;; Evaluation 
    (:module "eval"
             :depends-on ("data" "teaching")
             :components ((:file "evaluation-data")
                          (:file "evaluation" :depends-on ("evaluation-data"))))
    ))
