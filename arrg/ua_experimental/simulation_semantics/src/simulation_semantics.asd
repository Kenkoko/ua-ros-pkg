(in-package :asdf)

(defsystem "simulation_semantics"
    :components ((:file "pkg")

                 (:file "xml" :depends-on ("pkg")) ;; TODO: Remove
                 (:file "utils" :depends-on ("pkg"))
                 (:file "classes" :depends-on ("pkg"))

                 (:file "entity" :depends-on ("utils" "xml"))
                 (:file "robot" :depends-on ("entity"))
                 (:file "object-class" :depends-on ("entity"))
                 (:file "goals" :depends-on ("entity" "classes"))

                 (:file "simulator-class" :depends-on ("object-class"))                 

                 (:file "state" :depends-on ("classes"))   ;; TODO: Remove              
                 ;(:file "data" :depends-on ("classes"))
                 (:file "time-series" :depends-on ("classes"))
                 (:file "trace-to-intervals" :depends-on ("time-series"))

                 ;; TODO: Rename verb-learning
                 (:file "environment" :depends-on ("utils"))
                 (:file "verb-learning" :depends-on ("time-series" "environment"))
                 (:file "scenarios" :depends-on ("verb-learning"))
                 (:file "evaluation-data" :depends-on ("scenarios"))
                 (:file "evaluation" :depends-on ("evaluation-data"))
                 (:file "teaching" :depends-on ("verb-learning")))

    :depends-on (roslisp
                 cl-transforms
                 ;"actionlib" ;; Don't think we need this anymore?
                 "gbbopen-ros"
                 ;; Messages and services
                 "simulation_semantics-srv"
                 "oomdp_msgs-msg"
                 "oomdp_msgs-srv"
                 "verb_learning-msg"
                 "verb_learning-srv"
                 "time_series-msg"
                 "time_series-srv"
                 "gazebo-msg"
                 "gazebo-srv" 
                 "gazebo_plugins-msg"
                 "gazebo_plugins-srv" 
                 "geometry_msgs-msg"
                 "std_srvs-srv"
                 "std_msgs-msg"
                 "wubble_description-srv"
                 "move_base_msgs-msg"
                 "plotter-srv"))
