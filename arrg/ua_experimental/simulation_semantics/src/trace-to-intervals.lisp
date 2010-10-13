(in-package :simsem)

;; This is a separate node for converting sequences of MDPStates to Intervals for time_series

(def-service-callback ConvertToEpisode (states) 
  (make-response :episode (intervals-to-episode (pmts-to-intervals (convert-trace-to-pmts states nil)))))
    
(defun convert-to-episode-server ()
  (with-ros-node ("convert_to_episode_server" :spin t)
    (register-service "convert_to_episode" 'ConvertToEpisode)))