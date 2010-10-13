(in-package :asdf)

(defsystem "simulation_semantics"
    :components ((:file "pkg")

                 (:file "xml" :depends-on ("pkg"))
                 (:file "utils" :depends-on ("pkg"))
                 (:file "classes" :depends-on ("pkg"))

                 (:file "entity" :depends-on ("utils" "xml"))
                 (:file "robot" :depends-on ("entity"))
                 (:file "object-class" :depends-on ("entity"))
                 (:file "goals" :depends-on ("entity" "classes"))

                 (:file "simulator-class" :depends-on ("object-class"))                 

                 (:file "state" :depends-on ("classes"))                 
                 (:file "data" :depends-on ("classes"))
                 (:file "time-series" :depends-on ("data"))
                 (:file "trace-to-intervals" :depends-on ("time-series"))

                 (:file "verb-learning" :depends-on ("time-series")))

    :depends-on ("roslisp" 
                 "actionlib"
                 "cl-transforms"
                 "gbbopen-ros"
                 ;; Messages and services
                 "simulation_semantics-srv"
                 "oomdp_msgs-msg"
                 "oomdp_msgs-srv"
                 "verb_learning-srv"
                 "verb_learning-msg"
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
