(in-package :simsem)

(defun perform-action (action)
  "Performs action (a string) and returns the resulting state."
  (oomdp_msgs-srv:new_state-val 
   (call-service "environment/perform_action" 
                 'oomdp_msgs-srv:PerformAction
                 :action action)))