(in-package :asdf)

(defsystem "simulation_semantics/simulation_semantics"
    :components ((:file "pkg")
                 (:file "xml" :depends-on ("pkg"))
                 (:file "object-class" :depends-on ("xml"))
                 (:file "state" :depends-on ("object-class"))
                 (:file "simulator-class" :depends-on ("object-class"))
                 (:file "simulators" :depends-on ("simulator-class")))
    :depends-on ("roslisp" 
                 "simulator_experiments-msg"
                 "simulator_experiments-srv"
                 "gazebo_plugins-srv" 
                 "gazebo_plugins-msg"
                 "geometry_msgs-msg"
                 "std_srvs-srv"
                 "std_msgs-msg"
                 "gbbopen/gbbopen"))
