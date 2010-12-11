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

(defun make-learning-curve (domain verb-word num-trials &key (min-training-size 1) (max-training-size nil))
  (let* ((training (make-training domain verb-word)))
    (loop for i from min-training-size upto (if max-training-size max-training-size (length training))
       collect (make-learning-point domain verb-word i num-trials))))

;; TODO: Fix name mapping
(defun make-learning-point (domain verb-word training-size num-trials)
  (let* ((training (make-training domain verb-word))
         (verb (find-verb verb-word)))
    (format t "Begin Testing for training size ~d...~%"  training-size) 
    (loop for j from 1 upto num-trials
       do (clear-verb-meaning verb) ;; Reset the signature, FSM 
         (format t "Begin Trial ~d with training size ~d~%" j training-size)
         (loop for instance in (sample-random-subset training training-size)
            for student-result = (run-test verb (first instance))
            if (eq :accepted (accepted-of student-result))
            do (update-verb-with-trace verb (trace-of student-result) (argument-names (first instance)))
            else if (eq :rejected (accepted-of student-result))
            do (update-verb-with-negative-trace verb (trace-of student-result) (argument-names (first instance)))
            do (format t "Presenting teacher's demonstration~%") 
              (present-training-instance verb-word instance))
         (format t "Testing robot performance...~%")
       collect (run-tests verb-word (make-tests domain verb-word)))))
                       
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
    
(defun run-tests (verb-word tests)
  (loop with verb = (find-verb verb-word)
     for object-states in tests
     for i from 1 upto (length tests)
     do (format t "Running test ~d...~%" i)
     collect (run-test verb object-states)))
     
(defun run-test (verb object-states)
  "Returns a list (planning-success planning-time plan-length trace accept)"
  (let* ((start-state (initialize object-states))
         (perform-response (call-service "verb_learning/perform_verb" 'verb_learning-srv:PerformVerb
                                         :verb (make-message "verb_learning/VerbInstance"
                                                            :verb (lexical-form-of verb)
                                                            :arguments (ros-list (argument-strings verb))
                                                            :bindings (ros-list (argument-names object-states)))
                                         :start_state start-state
                                         :execution_limit 30))
         (trace (verb_learning-srv:trace-val perform-response))
         (result (make-instance 'test-result
                                :execution-success (verb_learning-srv:execution_success-val perform-response)
                                :planning-time (/ (verb_learning-srv:planning_time-val perform-response) 
                                                  1000)
                                :execution-length (verb_learning-srv:execution_length-val perform-response)
                                :trace trace)))
    (cond ((> (length trace) 0)
           (sb-ext:run-program "/usr/bin/notify-send" '("Robot needs feedback"))
           (format t ">>> Accept student's performance? (t or nil)~%") 
           (setf (accepted-of result) (read-accepted))))
    result))
                                          

(defun seq-verb-test ()
  (load-verbs)
  (define-sequential-verb "go-through" '(agent waypoint goal) '(("go" agent waypoint)
                                                                ("go" agent goal))))
  ;(make-learning-point "go-through" 2 1))
