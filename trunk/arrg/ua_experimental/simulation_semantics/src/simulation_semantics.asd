(in-package :asdf)

(defsystem "simulation_semantics/simulation_semantics"
    :components ((:file "pkg")
                 (:file "objects" :depends-on ("pkg"))
                 (:file "simulators" :depends-on ("objects")))
    :depends-on ("roslisp" 
                 "simulator_experiments-srv" 
                 "gazebo_plugins-srv" 
                 "gazebo_plugins-msg"
                 "std_srvs-srv"
                 "std_msgs-msg"))
