(defpackage :simulation_semantics
  (:documentation "A platform to dynamically configure and control simulators, as well as record simulator state. Meant to support language learning under a simulation-based view of NL semantics")
  (:nicknames :simsem)
  (:shadowing-import-from :geometry_msgs-msg :point-val)
  (:shadowing-import-from :time_series-srv :episode-val)
  (:use :cl 
        :roslisp 
        :s-xml
        :std_msgs-msg
        :geometry_msgs-msg
        :time_series-msg
        :time_series-srv
        :simulation_semantics-srv
        :gbbopen 
        :gbbopen-tools)
  (:export :convert-to-episode-server))
