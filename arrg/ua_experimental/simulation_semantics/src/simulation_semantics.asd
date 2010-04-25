(in-package :asdf)

(defsystem "simulation_semantics/simulation_semantics"
    :components ((:file "pkg")
                 (:file "xml" :depends-on ("pkg"))
                 (:file "permutations" :depends-on ("pkg"))
                 (:file "object-class" :depends-on ("xml"))
                 (:file "simulator-class" :depends-on ("object-class"))
                 (:file "classes" :depends-on ("object-class" "simulator-class"))
                 (:file "state" :depends-on ("classes"))
                 (:file "predicates" :depends-on ("state" "permutations"))
                 (:file "simulators" :depends-on ("classes")))
    :depends-on ("roslisp" 
                 "simulator_experiments-msg"
                 "simulator_experiments-srv"
                 "gazebo_plugins-srv" 
                 "gazebo_plugins-msg"
                 "geometry_msgs-msg"
                 "std_srvs-srv"
                 "std_msgs-msg"
                 "gbbopen/gbbopen"))
