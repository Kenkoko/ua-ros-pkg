(in-package :simulation_semantics)

;;============================================================                

(defun get-object-state (name state)
  (loop for object-state across (oomdp_msgs-msg:object_states-val state)
     if (string-equal name (oomdp_msgs-msg:name-val object-state))
     do (return object-state)))

(defun get-relation (state rel-name &rest obj-names)
  "Returns the relation (msg) that matches rel-name and obj-names (in any order)"
  ;(print obj-names)
  (loop for rel across (oomdp_msgs-msg:relations-val state)
     if (and (string-equal (oomdp_msgs-msg:relation-val rel) rel-name)
             (= (length (intersection obj-names
                                      (vec-to-list (oomdp_msgs-msg:obj_names-val rel))
                                      :test #'string-equal))
                (length obj-names)))
     do (return rel)))
     
;;============================================================                

(defun find-object-by-gazebo-name (gazebo-name)
  (let* ((result (find-instances '(entity :plus-subclasses)
                                 '(object-library)
                                 (list 'is-equal 
                                       'gazebo-name 
                                       gazebo-name))))
    (if result (first result) nil)))

;;============================================================                

(defun print-relation (rel-msg)
  (let* ((rel-name (oomdp_msgs-msg:relation-val rel-msg))
         (obj-names (vec-to-list (oomdp_msgs-msg:obj_names-val rel-msg)))
         (value (oomdp_msgs-msg:value-val rel-msg)))
    (format t "~a(~{~a~^,~})=~a~%" rel-name obj-names (if value "true" "false"))))

(defun print-mdp-state (state)
  ;; TODO: Print object states
  (format t "~%MDP STATE:~%")
  (loop for obj-state across (oomdp_msgs-msg:object_states-val state)
     do (format t "~a~%" obj-state))
  (loop for relation across (oomdp_msgs-msg:relations-val state)
     do (print-relation relation)))