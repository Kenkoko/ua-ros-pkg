(in-package :simsem)

(defparameter *execution-limit* 30)

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

(defun make-learning-curve (domain verb-word num-trials 
                            &key (min-training-size 1) (max-training-size nil) (scoring-function 'human-scorer))
  (let* ((training (make-training domain verb-word)))
    (loop for i from min-training-size upto (if max-training-size max-training-size (length training))
       collect (make-learning-point domain verb-word i num-trials scoring-function))))

(defun make-learning-point (domain verb-word training-size num-trials scoring-function)
  (let* ((training (make-training domain verb-word))
         (verb (find-verb verb-word)))
         ;(training-sets (sample-n-perms training training-size num-trials))
         ;(actual-num-trials (length training-sets))) ;; because there may no be num-trials possible permutations
    (format t "Begin Evaluation for Training Size ~d...~%"  training-size) 
    (loop for j from 1 upto num-trials ;actual-num-trials
       for training-set = (sample-random-subset training training-size)  ;(nth (- j 1) training-sets)
       do (clear-verb-meaning verb) ;; Reset the signature, FSM 
         (format t "Begin Trial ~d of ~d with Training Size ~d~%" j num-trials training-size)
         ;(format t "~a~%" training-set)
         (loop for instance in training-set
            ;for student-result = (run-test verb (first instance) scoring-function)
            ;if (eq :accepted (accepted-of student-result))
            ;do (update-verb-with-trace verb (trace-of student-result) (argument-names (first instance)))
            ;else if (eq :rejected (accepted-of student-result))
            ;do (update-verb-with-negative-trace verb (trace-of student-result) (argument-names (first instance)))
            do (format t "Presenting teacher's demonstration~%") 
              (present-training-instance verb-word instance))
         (format t "Testing robot performance...~%")
       collect (run-tests verb-word (make-tests domain verb-word) scoring-function))))

(defun make-learning-trajectory (domain verb-word scoring-function)
  (let* ((training (make-training domain verb-word))
         (verb (find-verb verb-word))
         (training-set (sample-random-subset training (length training))))
    (format t "Clearing verb meaning...~%")
    (clear-verb-meaning verb)
    (format t "Begin Trajectory~%")
    (loop for instance in training-set
       for i from 1
       for student-result = (run-test verb (first instance) scoring-function)
       do (format t "~%============ Teaching Loop Iteration ~a~%" i)
       if (eq :accepted (accepted-of student-result))
       do (update-verb-with-trace verb (trace-of student-result) (argument-names (first instance)))
       else if (eq :rejected (accepted-of student-result))
       do (update-verb-with-negative-trace verb (trace-of student-result) (argument-names (first instance)))
       do (format t "Presenting teacher's demonstration~%") 
         (present-training-instance verb-word instance)
         (format t "Testing robot performance...~%")
       collect (list (run-tests verb-word (make-tests domain verb-word) scoring-function)))))

(defun process-learning-trajectories (trajectories)
  (process-learning-curve
   (loop for i from 0 below (length (first trajectories))
      collect (loop for traj in trajectories 
                 append (nth i traj)))))

(defun process-learning-curve (lc-result)
  (loop for training-size from 1 upto (length lc-result)
     for results in lc-result
     for avg-times = nil
     for avg-scores = nil
     for avg-lengths = nil
     do (format t "~%>>>>>>>>>>>>>~%Results with Training Size ~d~%" training-size)
       (loop for result-set in results
          for n = (length result-set)
          for percent-correct = (/ (count-if (lambda (res) (eq (accepted-of res) :accepted)) result-set) 
                                   n)
          for avg-planning-time = (/ (loop for result in result-set summing (planning-time-of result))
                                     n)
          for avg-execution-length = (/ (loop for result in result-set summing (execution-length-of result)) 
                                        n)
          do (format t "  Percent Correct: ~,2f~%" percent-correct)
            (format t "  Average Planning Time: ~,2f seconds~%" avg-planning-time)
            (format t "  Average Execution Length: ~,2f~%" avg-execution-length)
            (push avg-planning-time avg-times)
            (push percent-correct avg-scores)
            (push avg-execution-length  avg-lengths))
       (format t "Average Percent Correct: ~,2f (stddev ~,2f)~%" (mean avg-scores)
               (standard-deviation avg-scores))
       (format t "Average Planning Time: ~,2f seconds (stddev ~,2f)~%" (mean avg-times)
               (standard-deviation avg-times))
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
  (format t "Testing Student...~%")
  (let* ((start-state (initialize object-states))
         (perform-response (call-service "verb_learning/perform_verb" 'verb_learning-srv:PerformVerb
                                         :verb (make-message "verb_learning/VerbInstance"
                                                            :verb (lexical-form-of verb)
                                                            :arguments (ros-list (argument-strings verb))
                                                            :bindings (ros-list (argument-names object-states)))
                                         :start_state start-state
                                         :execution_limit *execution-limit*))
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
;; Automatic Scorers - TODO: Move these to a separate file

(defun human-scorer (result)
  (declare (ignore result))
  (sb-ext:run-program "/usr/bin/notify-send" '("Robot needs feedback"))
  (format t ">>> Accept student's performance?~%")
  (read-accepted))

(defun ww2d-go-scorer (result)
  (format t "~%~%+++++++++++++++++++++++++++++++++~%~%")

  (loop with trace = (verb_learning-srv:trace-val result)
     with step = 'approaching
     for state across trace
     for in-contact? = (contains-relation state "Collision" '("person" "place"))

     if (eq step 'approaching)
     do (if in-contact?
            (setf step 'contact))

     else if (eq step 'contact)
     do (if (not in-contact?)
            (return-from ww2d-go-scorer :rejected))

     finally (return (if (eq step 'contact)
                         :accepted
                         :neither))))

(defun ww2d-intercept-scorer (result)
  (format t "~%~%+++++++++++++++++++++++++++++++++~%~%")

  (loop with trace = (verb_learning-srv:trace-val result)
     for state across trace
     for met-enemy? = (contains-relation state "Collision" '("person" "enemy"))
     for enemy-at-goal? = (contains-relation state "Collision" '("enemy" "place"))
     do (cond (enemy-at-goal?
               (return-from ww2d-intercept-scorer :rejected))
              (met-enemy?
               (return-from ww2d-intercept-scorer :accepted))))
  :neither)

(defun gz-go-scorer (result)
  (loop with trace = (verb_learning-srv:trace-val result)
     with step = 'approaching
     for state across trace
     for in-contact? = (contains-relation state "Contact" '("robot_description" "goal"))
     if (and in-contact? (eq step 'approaching))
     do (setf step 'contact)
     else if (and (not in-contact?) (eq step 'contact))
     do (return-from gz-go-scorer :rejected)
     finally (return (if (eq step 'contact)
                         :accepted
                         :neither))))

(defun gz-deliver-scorer (result)
  (loop with trace = (verb_learning-srv:trace-val result)
     with step = 'approaching
     for state across trace
     for robot-at-item? = (contains-relation state "Contact" '("robot_description" "item"))
     for robot-has-item? = (contains-relation state "Carrying" '("robot_description" "item"))
     for robot-at-goal? = (contains-relation state "Contact" '("robot_description" "destination"))
     do (cond ((eq step 'approaching)
               (cond (robot-has-item? 
                      (setf step 'carrying))
                     (robot-at-item?
                      (setf step 'at-item))))

              ((eq step 'at-item)
               (cond (robot-has-item?
                      (setf step 'carrying))
                     ((not robot-at-item?)
                      (return-from gz-deliver-scorer :rejected)))) ;; left the item without picking it up 
                                       
              ((eq step 'carrying)
               (if (not robot-has-item?) 
                   (if robot-at-goal?
                       (setf step 'delivered)
                       (return-from gz-deliver-scorer :rejected)))) ;; dropped the item away from the goal
               
              ((eq step 'delivered)
               (if (not robot-at-goal?)
                   (return-from gz-deliver-scorer :rejected))) ;; left the goal after delivery
              )
     finally (progn
               (format t ">>>>> STEP: ~a~%" step)
               (if (eq step 'delivered)
                 (return :accepted)
                 (return :neither))))
)

;;==============================================================================


(defun nb-scorer (scorer)
  (lambda (result)
    (if (eq (funcall scorer result) :accepted)
        :accepted
        :rejected)))

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
          