(in-package :simsem)

;;===================================================================

(define-unit-class test-result ()
  (planning-success
   planning-time
   plan-length
   (trace :initform nil)
   (accepted :initform nil)))

(defun print-test-result (result)
  (format t "~a: ~a ~,2f ~a ~a~%" 
          (if (accepted-of result) "PASSED" "FAILED")
          (planning-success-of result)
          (planning-time-of result)
          (plan-length-of result)
          (not (null (trace-of result)))))
                    

;;===================================================================

(defun make-learning-curve (verb-word)
  (loop with training = (make-training verb-word)
     with verb = (find-verb verb-word)
     for i from 1 upto 3 ;(length training)
     for num-trials = (min 3 (unordered-permutations (length training) i))
     do (format t "Begin Testing...~%") 
     collect (loop for j from 1 upto num-trials
                do (clear-verb-meaning verb) ;; Reset the signature, FSM 
                  (format t "Begin Trial ~d with training size ~d~%" j i)
                  (loop for instance in (sample-random-subset training i)
                     for student-result = (run-test verb (first instance))
                     if (accepted-of student-result) do (update-verb-with-trace 
                                                         verb 
                                                         (trace-of student-result))
                     do (format t "Presenting teacher's demonstration~%") 
                       (present-training-instance verb-word instance))
                  (format t "Testing robot performance...~%")
                collect (run-tests verb-word (make-tests verb-word)))))
                       
(defun process-learning-curve (lc-result)
  (loop for training-size from 1 upto (length lc-result)
     for results in lc-result
     for avg-times = nil
     for avg-scores = nil
     for avg-lengths = nil
     do (format t "~%>>>>>>>>>>>>>~%Results with Training Size ~d~%" training-size)
       (loop with times = nil
          with scores = nil
          with lengths = nil
          for result-set in results
          do (loop for result in result-set
                for n = (length result-set)
                ;do (print-test-result result)
                if (accepted-of result) sum 1 into correct
                sum (planning-time-of result) into time-sum
                sum (plan-length-of result) into length-sum
                finally (push (/ time-sum n) times)
                  (push (/ correct n) scores)
                  (push (/ length-sum n) lengths))
            (format t "  Average Planning Time: ~,2f seconds~%" (mean times))
            (format t "  Percent Correct: ~,2f~%" (mean scores))
            (format t "  Average Plan Length: ~,2f~%" (mean lengths))
            (push (mean times) avg-times)
            (push (mean scores) avg-scores)
            (push (mean lengths) avg-lengths))
       (format t "Average Planning Time: ~,2f seconds (stddev ~,2f)~%" (mean avg-times)
               (standard-deviation avg-times))
       (format t "Average Percent Correct: ~,2f (stddev ~,2f)~%" (mean avg-scores)
               (standard-deviation avg-scores))
       (format t "Average Plan Length: ~,2f (stddev ~,2f)~%" (mean avg-lengths)
               (standard-deviation avg-lengths))))
                         
(defun present-training-instance (verb-word instance)
  (let* ((verb (find-verb verb-word))
         (start-state (initialize (first instance)))
         (new-trace (loop for action in (second instance)
                       do (format t "Performing action ~a~%" action)
                       collect (perform-action action) into trace
                       do (format t "Action complete.~%")
                       finally (return (push start-state trace))))
         (new-episode (trace-to-episode new-trace (instance-name-map (first instance) verb))))
    (update-canonical-signature new-episode :verb verb)))
    
(defun run-tests (verb-word tests)
  (loop with verb = (find-verb verb-word)
     for object-states in tests
     for i from 1 upto (length tests)
     do (format t "Running test ~d...~%" i)
     collect (run-test verb object-states)))
     
(defun run-test (verb object-states)
  "Returns a list (planning-success planning-time plan-length trace accept)"
  (let* ((start-state (initialize object-states))
         (planning-response (call-service "verb_learning/plan_verb" 'verb_learning-srv:PlanVerb
                                        :verb (make-message "verb_learning/VerbInstance"
                                                            :verb (lexical-form-of verb)
                                                            :arguments (ros-list (argument-strings verb))
                                                            :bindings (ros-list (argument-names object-states)))
                                        :start_state start-state))
         (result (make-instance 'test-result
                                :planning-success (verb_learning-srv:success-val planning-response)
                                :planning-time (/ (verb_learning-srv:time_millis-val planning-response) 
                                                  1000)
                                :plan-length (length (verb_learning-msg:actions-val 
                                                      (verb_learning-srv:plan-val planning-response))))))
    (if (planning-success-of result) 
        (progn (setf (trace-of result)
                     (execute-plan (verb_learning-srv:plan-val planning-response) start-state))
               (if (trace-of result)
                   (progn 
                     (sb-ext:run-program "/usr/bin/notify-send" '("Robot needs feedback"))
                     (format t ">>> Accept student's performance? (t or nil)~%") 
                     (setf (accepted-of result) (read)))
                   (format t "Plan execution failed.~%"))
               (progn
                 (format t "Planning failed.~%")
                 (setf (plan-length-of result) 20))))
    result))

;; How to measure time for future reference
;(let* ((t1 (get-internal-real-time)))
;          (sleep 1.03)
;          (format t "~,4fs~%" (/ (- (get-internal-real-time) t1)
;                                 internal-time-units-per-second)))

;;===================================================================

;; TODO: Need to replan if necessary
(defun execute-plan (plan start-state)
  "Executes the plan and returns a trace, or nil if the plan did not succeed"
  (let* ((states (verb_learning-msg:states-val plan))
         (actions (verb_learning-msg:actions-val plan)))
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
       do (format t "State did at Step ~d not match, terminating plan!~%" step)
         (print-mdp-state state)
         (print-mdp-state true-state)
       and return nil
       finally (return (push start-state trace)))))

(defun argument-names (object-states)
  (loop for object-state across object-states
     collect (oomdp_msgs-msg:name-val object-state)))           

;; NB: Does not preserve ordering!
(defun sample-random-subset (seq size)
  (loop with remaining = seq
     for i below size
     for chosen = (nth (random (length remaining)) remaining)
     do (setf remaining (remove chosen remaining))
     collect chosen))