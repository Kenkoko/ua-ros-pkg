(in-package :simsem)

(defun initialize (object-vector)
  (oomdp_msgs-srv:start_state-val
   (call-service "environment/initialize" 'oomdp_msgs-srv:InitializeEnvironment
                 :object_states object-vector)))

(defun perform-action (action)
  "Performs action (a string) and returns the resulting state."
  (oomdp_msgs-srv:new_state-val 
   (call-service "environment/perform_action" 
                 'oomdp_msgs-srv:PerformAction
                 :action action)))