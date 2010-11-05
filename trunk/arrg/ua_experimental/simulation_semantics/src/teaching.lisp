(in-package :simsem)

;;=====================================================================
;; Demonstrations (needs more methods)

(defun simulate-scenarios (&key (verb *current-verb*))
  (loop for scenario in (scenarios-of verb)
     do (run-simulator (simulator-of scenario))))

;;=====================================================================
;; Talking to verb_learning

(defun begin-verb-lesson (verb arguments)
  (format t "Lesson begun for verb: ~a(~{~a~^,~})~%" verb arguments)
  (setf *current-verb* (make-instance 'verb
                                      :instance-name (format nil "~a~a" verb "-verb")
                                      :lexical-form verb
                                      :argument-structure arguments)))

(defun load-verbs ()
  (loop with verbs = (verb_learning-srv:verbs-val 
                      (call-service "verb_learning/load_verbs" 'verb_learning-srv:LoadVerbs))
     for verb-desc across verbs
     for word = (verb_learning-msg:verb-val verb-desc)
     unless (find-verb word)
     collect (make-instance 'verb 
                            :instance-name (format nil "~a~a" word "-verb")
                            :lexical-form word
                            :argument-structure (vec-to-list (verb_learning-msg:arguments-val verb-desc)))))

(defun clear-verb-meaning (verb)
  (call-service "verb_learning/forget_verb" 'verb_learning-srv:ForgetVerb
                :verb (make-message "verb_learning/VerbDescription"
                                    :verb (lexical-form-of verb)
                                    :arguments (ros-list (argument-strings verb)))))

;(defun learn-canonical-signature (&key (verb *current-verb*))
;  (call-service "verb_learning/find_signature" 'verb_learning-srv:FindSignature
;                :episodes (get-episodes verb)
;                :verb (make-message "verb_learning/VerbDescription"
;                                    :verb (lexical-form-of verb)
;                                    :arguments (ros-list (argument-strings verb)))))

#+ignore(defun update-canonical-signature (episode &key (verb *current-verb*))
  (call-service "verb_learning/update_verb" 'verb_learning-srv:UpdateVerb
                :trace (list episode)
                :verb (make-message "verb_learning/VerbDescription"
                                    :verb (lexical-form-of verb)
                                    :arguments (ros-list (argument-strings verb)))))

(defun update-verb-with-trace (verb trace bindings)
  (call-service "verb_learning/update_verb" 'verb_learning-srv:UpdateVerb
                :is_positive t
                :trace trace
                :verb (make-message "verb_learning/VerbInstance"
                                    :verb (lexical-form-of verb)
                                    :arguments (ros-list (argument-strings verb))
                                    :bindings (ros-list bindings))))

(defun update-verb-with-negative-trace (verb trace bindings)
  (call-service "verb_learning/update_verb" 'verb_learning-srv:UpdateVerb
                :is_positive nil
                :trace trace
                :verb (make-message "verb_learning/VerbInstance"
                                    :verb (lexical-form-of verb)
                                    :arguments (ros-list (argument-strings verb))
                                    :bindings (ros-list bindings))))

;;=====================================================================
;; Modifying verbs

(defun constrain-verb ()
  (call-service "verb_learning/constrain_verb" 'verb_learning-srv:ConstrainVerb
                :verb "go-around"
                :banned_props (list "Contact(thing,obstacle)")))

;;=====================================================================
;; Make it happen!

;; TODO: These are deprecated for now, need to shift to new scenario format

#+ignore(defun perform-verb (&key (verb *current-verb*))
  (loop for scenario in (scenarios-of verb)
     for i from 1
     do (format t "Scenario ~d:~%" i)
       (loop for object in (argument-list-of scenario)
          for arg in (argument-structure-of verb)
          do (format t "~a => ~a at ~a~%" arg (class-name (class-of object))
                     (if (gethash object (objects-of (simulator-of scenario)))
                         (gethash object (objects-of (simulator-of scenario)))
                         (position-of (first (goals-of (simulator-of scenario))))))))
  (let* ((scenario (nth (- (read) 1) (scenarios-of verb))))
    (format t "Planning for selected scenario...~%")
    (killall)
    (construct (simulator-of scenario))
    (let* ((start-state (oomdp_msgs-srv:start_state-val 
                         (call-service "environment/initialize" 'oomdp_msgs-srv:InitializeEnvironment
                                       :object_states #())))
           (planning-result (call-service "verb_learning/plan_verb" 'verb_learning-srv:PlanVerb
                                          :verb (make-verb-instance verb scenario)
                                          :start_state start-state)))
      (format t "~a~%" (verb_learning-srv:success-val planning-result))
      (if (verb_learning-srv:success-val planning-result)
          (let* ((new-trace (execute-plan (verb_learning-srv:plan-val planning-result) start-state))
                 (new-episode (if new-trace (trace-to-episode new-trace (name-map-of scenario verb)))))
            (if new-episode
                (progn 
                  (format t "Add episode to verb signature? (t or nil)~%")
                  (if (read) (update-canonical-signature new-episode :verb verb)))
                (format t "Plan execution failed.~%")))
            ;; TODO: Allow for demonstration here
          (format t "Planning failed.~%")))))

;; Some help from teacher

#+ignore
(defun demonstrate-verb (&key (verb *current-verb*))
  (loop for scenario in (scenarios-of verb)
     for i from 1
     do (format t "Scenario ~d:~%" i)
       (loop for object in (argument-list-of scenario)
          for arg in (argument-structure-of verb)
          do (format t "~a => ~a at ~a~%" arg (class-name (class-of object))
                     (if (gethash object (objects-of (simulator-of scenario)))
                         (gethash object (objects-of (simulator-of scenario)))
                         (position-of (first (goals-of (simulator-of scenario))))))))
  (let* ((scenario (nth (- (read) 1) (scenarios-of verb))))
    (format t "Setting up scenario...~%")
    (killall)
    (construct (simulator-of scenario))
    (format t "Please enter action string or nil to stop~%")
    (let* ((start-state (oomdp_msgs-srv:start_state-val 
                         (call-service "environment/initialize" 'oomdp_msgs-srv:InitializeEnvironment
                                       :object_states #())))
           (new-trace (loop with action = (read)
                         until (null action)
                         do (format t "Performing action ~a~%" action)
                         collect (perform-action action) into trace
                         do (format t "Action complete.~%")
                           (setf action (read))
                         finally (return (push start-state trace))))
           (new-episode (trace-to-episode new-trace (name-map-of scenario verb))))
      (format t "Add episode to verb signature? (t or nil)~%")
      (if (read) (update-canonical-signature new-episode :verb verb)))))
