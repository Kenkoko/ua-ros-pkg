(in-package :asdf)

(defsystem "simulation_semantics"
    :components ((:file "pkg")

                 (:file "xml" :depends-on ("pkg"))
                 (:file "permutations" :depends-on ("xml"))
                 (:file "utils" :depends-on ("permutations"))

                 (:file "thing-class" :depends-on ("utils"))
                 (:file "robot" :depends-on ("thing-class"))
                 (:file "object-class" :depends-on ("thing-class"))

                 (:file "goals" :depends-on ("thing-class" "classes"))
                 (:file "simulator-class" :depends-on ("object-class" "utils"))
                 (:file "classes" :depends-on ("object-class" "simulator-class"))
                 (:file "state" :depends-on ("classes"))
                 (:file "predicates" :depends-on ("object-class" "state" "permutations"))

                 (:file "data" :depends-on ("classes"))
                 (:file "simulators" :depends-on ("classes"))
                 (:file "online" :depends-on ("simulators"))
                 (:file "manipulation" :depends-on ("simulators"))

                 (:file "analysis" :depends-on ("simulators"))
                 (:file "time-series" :depends-on ("analysis"))
                 (:file "verb-learning" :depends-on ("time-series")))

    :depends-on ("roslisp" 
                 :gbbopen-ros
                 "cl-transforms"
                 "plotter-srv"
                 "simulator_experiments-msg"
                 "simulator_experiments-srv"
                 "gazebo-msg"
                 "gazebo-srv" 
                 "gazebo_plugins-msg"
                 "gazebo_plugins-srv" 
                 "geometry_msgs-msg"
                 "std_srvs-srv"
                 "std_msgs-msg"
                 "time_series-msg"
                 "time_series-srv"))
