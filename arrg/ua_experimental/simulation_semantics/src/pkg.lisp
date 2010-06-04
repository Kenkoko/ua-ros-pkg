(defpackage :simulation_semantics
  (:documentation "A platform to dynamically configure and control simulators, as well as record simulator state. Meant to support language learning under a simulation-based view of NL semantics")
  (:nicknames :simsem)
  (:use :cl 
        :roslisp 
        :s-xml
        :std_srvs-srv 
        :std_msgs-msg
        :geometry_msgs-msg
        ;:gazebo-srv
        :simulator_experiments-srv
        :gbbopen
        :gbbopen-tools))
