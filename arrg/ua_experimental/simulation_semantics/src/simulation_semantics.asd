(in-package :asdf)

(defsystem "simulation_semantics/simulation_semantics"
    :components ((:file "pkg")

                 (:file "xml" :depends-on ("pkg"))
                 (:file "permutations" :depends-on ("pkg"))

                 (:file "thing-class" :depends-on ("xml"))
                 (:file "robot" :depends-on ("thing-class"))
                 (:file "object-class" :depends-on ("thing-class"))

                 (:file "simulator-class" :depends-on ("object-class"))
                 (:file "classes" :depends-on ("object-class" "simulator-class"))
                 (:file "state" :depends-on ("classes"))
                 (:file "predicates" :depends-on ("object-class" "state" "permutations"))

                 (:file "simulators" :depends-on ("classes"))
                 (:file "online" :depends-on ("simulators"))
                 (:file "manipulation" :depends-on ("simulators")))
    :depends-on ("roslisp" 
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
                 "gbbopen/gbbopen"))
