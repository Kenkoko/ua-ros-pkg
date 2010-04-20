(in-package :asdf)

(defsystem "simulation_semantics/simulation_semantics"
    :components ((:file "pkg")
                 (:file "object-class" :depends-on ("pkg"))
                 (:file "simulator-class" :depends-on ("object-class"))
                 (:file "simulators" :depends-on ("simulator-class")))
    :depends-on ("roslisp" 
                 "simulator_experiments-srv" 
                 "gazebo_plugins-srv" 
                 "gazebo_plugins-msg"
                 "geometry_msgs-msg"
                 "std_srvs-srv"
                 "std_msgs-msg"))
