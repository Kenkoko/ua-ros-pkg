(in-package :simsem)

;;===================================================================

(define-unit-class test-result ()
  (execution-success
   planning-time
   execution-length
   (trace :initform nil)
   (accepted :initform :neither)))

(defun read-accepted ()
  (format t "Choose one of: (a)ccept, (r)eject, (n)either.~%")
  (let* ((input (read)))
    (cond ((eq input 'a)
           :accepted)
          ((eq input 'r)
           :rejected)
          (t
           :neither))))
           
(defun print-test-result (result)
  (format t "~a: ~a ~,2f ~a ~a~%" 
          (accepted-of result)
          (execution-success-of result)
          (planning-time-of result)
          (execution-length-of result)
          (not (null (trace-of result)))))
                 
;;===================================================================

(defun make-learning-curve (domain verb-word num-trials &key (min-training-size 1) (max-training-size nil) (scoring-function 'human-scorer))
  (let* ((training (make-training domain verb-word)))
    (loop for i from min-training-size upto (if max-training-size max-training-size (length training))
       collect (make-learning-point domain verb-word i num-trials scoring-function))))

(defun make-learning-point (domain verb-word training-size num-trials scoring-function)
  (let* ((training (make-training domain verb-word))
         (verb (find-verb verb-word))
         (training-sets (sample-n-perms training training-size num-trials))
         (actual-num-trials (length training-sets))) ;; because there may no be num-trials possible permutations
    (format t "Begin Evaluation for Training Size ~d...~%"  training-size) 
    (loop for j from 1 upto actual-num-trials
       for training-set = (nth (- j 1) training-sets)
       do (clear-verb-meaning verb) ;; Reset the signature, FSM 
         (format t "Begin Trial ~d of ~d with Training Size ~d~%" j actual-num-trials training-size)
         ;(format t "~a~%" training-set)
         (loop for instance in training-set
            for student-result = (run-test verb (first instance) scoring-function)
            if (eq :accepted (accepted-of student-result))
            do (update-verb-with-trace verb (trace-of student-result) (argument-names (first instance)))
            else if (eq :rejected (accepted-of student-result))
            do (update-verb-with-negative-trace verb (trace-of student-result) (argument-names (first instance)))
            do (format t "Presenting teacher's demonstration~%") 
              (present-training-instance verb-word instance))
         (format t "Testing robot performance...~%")
       collect (run-tests verb-word (make-tests domain verb-word) scoring-function))))
                       
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
                if (eq (accepted-of result) :accepted) sum 1 into correct
                sum (planning-time-of result) into time-sum
                sum (execution-length-of result) into length-sum
                finally (push (/ time-sum n) times)
                  (push (/ correct n) scores)
                  (push (/ length-sum n) lengths))
            (format t "  Average Planning Time: ~,2f seconds~%" (mean times))
            (format t "  Percent Correct: ~,2f~%" (mean scores))
            (format t "  Average Execution Length: ~,2f~%" (mean lengths))
            (push (mean times) avg-times)
            (push (mean scores) avg-scores)
            (push (mean lengths) avg-lengths))
       (format t "Average Planning Time: ~,2f seconds (stddev ~,2f)~%" (mean avg-times)
               (standard-deviation avg-times))
       (format t "Average Percent Correct: ~,2f (stddev ~,2f)~%" (mean avg-scores)
               (standard-deviation avg-scores))
       (format t "Average Execution Length: ~,2f (stddev ~,2f)~%" (mean avg-lengths)
               (standard-deviation avg-lengths))))
                         
(defun present-training-instance (verb-word instance)
  (let* ((verb (find-verb verb-word))
         (start-state (initialize (first instance)))
         (teacher-trace (loop for action in (second instance)
                       do (format t "Performing action ~a~%" action)
                       collect (perform-action action) into trace
                       do (format t "Action complete.~%")
                       finally (return (push start-state trace)))))
    (update-verb-with-trace verb teacher-trace (argument-names (first instance)))))
    
(defun run-tests (verb-word tests scoring-function)
  (loop with verb = (find-verb verb-word)
     for object-states in tests
     for i from 1 upto (length tests)
     do (format t "Running test ~d...~%" i)
     collect (run-test verb object-states scoring-function)))
     
(defun run-test (verb object-states scoring-function)
  "Returns a list (planning-success planning-time plan-length trace accept)"
  (let* ((start-state (initialize object-states))
         (perform-response (call-service "verb_learning/perform_verb" 'verb_learning-srv:PerformVerb
                                         :verb (make-message "verb_learning/VerbInstance"
                                                            :verb (lexical-form-of verb)
                                                            :arguments (ros-list (argument-strings verb))
                                                            :bindings (ros-list (argument-names object-states)))
                                         :start_state start-state
                                         :execution_limit 50))
         (trace (verb_learning-srv:trace-val perform-response))
         (result (make-instance 'test-result
                                :execution-success (verb_learning-srv:execution_success-val perform-response)
                                :planning-time (/ (verb_learning-srv:planning_time-val perform-response) 
                                                  1000)
                                :execution-length (verb_learning-srv:execution_length-val perform-response)
                                :trace trace)))
    (cond ((> (length trace) 1)          
           (setf (accepted-of result)
                 (funcall scoring-function perform-response))))
    (format t "Test Result: ~a~%" (accepted-of result))
    result))                                        

;;============================================================================
;; Automatic Scorers

(defun human-scorer (result)
  (declare (ignore result))
  (sb-ext:run-program "/usr/bin/notify-send" '("Robot needs feedback"))
  (format t ">>> Accept student's performance?~%")
  (read-accepted))

(defun simple-gz-go-scorer (result)
  (let* ((trace (verb_learning-srv:trace-val result))
         (last-state (aref trace (- (length trace) 1))))
    (format t "=========================~%SCORING~%")
    (loop with contact-count = 0
       for relation across (oomdp_msgs-msg:relations-val last-state)
       do (format t "~a~%" relation)
       if (string-equal "Contact" (oomdp_msgs-msg:relation-val relation))
       do (incf contact-count)
       finally (return (if (eq contact-count 2)
                           :accepted
                           :neither)))))

;; TODO: Is there an issue with the last deliver test?
;; TODO: What if the robot starts at the object - will our visit-counts work?
(defun gz-deliver-scorer (result)
  (let* ((trace (vec-to-list (verb_learning-srv:trace-val result)))
         (actions (vec-to-list (verb_learning-srv:actions-val result))))
    (format t "~a~%" actions)
    ;; Reject if more/less than one drop
    ;; Actually, this is too strong, drop could be performed with nothing in hand
    ;(unless (eq 1 (count "drop" actions :test 'string-equal))
    ; (return-from gz-deliver-scorer :rejected))
    ;; Reject if continues past the drop
    (unless (string-equal "drop" (nth (- (length actions) 2) actions))
      (return-from gz-deliver-scorer :rejected))
    ;; TODO: Check last state is drop at goal state
    ;; TODO: Check no revisit of object
    (loop with was-holding = nil
       with holding-count = 0
       with was-at-object = nil
       with object-visit-count = 0
       with was-at-goal = nil
       with goal-visit-count = 0
       for state in trace
       for action in actions
       for is-holding = (contains-relation state "Carrying" '("robot_description" "item"))
       for is-at-object = (contains-relation state "Contact" '("robot_description" "item"))
       for is-at-goal = (contains-relation state "Contact" '("item" "destination"))
       do (cond ((and is-holding (not was-holding))
                 (incf holding-count)
                 (if (> holding-count 1)
                     (return-from gz-deliver-scorer :rejected))))
         (cond ((and is-at-object (not was-at-object))
                (incf object-visit-count)
                (if (> object-visit-count 1)
                    (return-from gz-deliver-scorer :rejected))))
         (cond ((and (and is-at-goal (not was-at-goal))
                     (> holding-count 0)) ;; Only count if we have already picked up the object 
                (incf goal-visit-count)
                (if (> goal-visit-count 1)
                    (return-from gz-deliver-scorer :rejected))
                (unless (string-equal action "drop")
                  (return-from gz-deliver-scorer :rejected))))
         (setf was-holding is-holding)
         (setf was-at-object is-at-object)
         (setf was-at-goal is-at-goal)
       finally (if (or (eq goal-visit-count 0)
                       (eq object-visit-count 0)
                       (eq holding-count 0))
                   (return-from gz-deliver-scorer :neither)))
    :accepted))

(defun contains-relation (state rel-name names)
  (loop for relation across (oomdp_msgs-msg:relations-val state)
     if (matches-relation relation rel-name names)
     do (return-from contains-relation t))
  nil)

(defun matches-relation (rel rel-name names)
  (and (string-equal (oomdp_msgs-msg:relation-val rel) rel-name)
       (eq (length (oomdp_msgs-msg:obj_names-val rel)) (length names))
       (loop with result = (oomdp_msgs-msg:value-val rel)
          for name in names 
          for name2 across (oomdp_msgs-msg:obj_names-val rel)
          if (not (string-equal name name2))
          do (setf result nil)
          finally (return result))))
          