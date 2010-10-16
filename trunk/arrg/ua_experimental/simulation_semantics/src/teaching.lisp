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

(defun learn-canonical-signature (&key (verb *current-verb*))
  (call-service "verb_learning/find_signature" 'verb_learning-srv:FindSignature
                :episodes (get-episodes verb)
                :verb (make-message "verb_learning/VerbDescription"
                                    :verb (lexical-form-of verb)
                                    :arguments (ros-list (argument-strings verb)))))

(defun update-canonical-signature (episode &key (verb *current-verb*))
  (call-service "verb_learning/update_signature" 'verb_learning-srv:FindSignature
                :episodes (list episode)
                :verb (make-message "verb_learning/VerbDescription"
                                    :verb (lexical-form-of verb)
                                    :arguments (ros-list (argument-strings verb)))))

;; TODO: Fix this function
#+ignore(defun learn-counterfactual-signature (&key (verb *current-verb*))
  (call-service "verb_learning/find_signature" 'time_series-srv:FindSignature
                :episodes (get-intervention-episodes verb) 
                :verb (concatenate 'string (lexical-form-of verb) "-counterfactual")))

;;=====================================================================
;; Modifying verbs

(defun constrain-verb ()
  (call-service "verb_learning/constrain_verb" 'verb_learning-srv:ConstrainVerb
                :verb "go-around"
                :banned_props (list "Contact(thing,obstacle)")))

#+ignore(defun specialize-verb (verb parent new-arguments)
  (let* ((parent-verb (find-verb parent)))
    (if (not parent-verb)
        (format "PARENT DOES NOT EXIST~%"))
    (setf *current-verb* (make-instance 'verb
                                        :instance-name (format nil "~a~a" verb "-verb")
                                        :lexical-form verb
                                        :argument-structure new-arguments))
    (call-service "verb_learning/specialize_verb" 'verb_learning-srv:SpecializeVerb
                  :parent (make-verb-description parent-verb)
                  :new_verb (make-verb-description *current-verb*))))
;; TODO: Implement specialize on the other end.

;;=====================================================================
;; Make it happen!

(defun perform-verb (&key (verb *current-verb*))
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

;; TODO: Need to replan if necessary
(defun execute-plan (plan start-state)
  "Executes the plan and returns an episode, or nil if the plan did not succeed"
  (let* ((states (verb_learning-msg:states-val plan))
         (actions (verb_learning-msg:actions-val plan)))
    ;(format t "PLAN:~%~a~%" plan)
    (loop with true-state = start-state
       for state across states
       for action across actions 
       for step from 1
       if (msg-equal state true-state)
       do (format t "State matched for Step ~d, proceeding with plan!~%" step)
         (cond ((string-equal action "TERMINATE")
                (format t "Action is TERMINATE, plan execution is complete!~%"))
               (t 
                (format t "Performing action: ~a~%" action)
                (setf true-state (perform-action action))))
       and collect true-state into trace
       else 
       do (format t "State did at Step ~d not match, terminating plan!~%~a~%~a~%" step state true-state)
       and return nil
       finally (return (push start-state trace)))))

;; Some help from teacher

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
