(in-package :simulation_semantics)

;; This buffer is for notes you don't want to save, and for Lisp evaluation.
;; If you want to create a file, visit that file with C-x C-f,
;; then enter the text in that file's own buffer.

(defun match-predicate (target-pred pred-name objects)
  ;(format t "~a ~a ~a~%" target-pred pred-name objects)
  (if (eq (first target-pred) pred-name)
      (let ((match-index (search objects (rest target-pred) :test 'equal)))
        (if match-index
            (zerop match-index)))))

(defun simple-match-predicate (pred-spec target-pred)
  "tests whether target-pred matches pred-spec"
  (search pred-spec target-pred :test 'equal))

(defun get-matching-predicates (description simulation)
  (let* ((ws (first (find-instances 'world-state simulation :all))))
    (match-predicates-in-state description ws)))

(defun match-predicates-in-state (description ws)
  (loop for predicate in (predicates-of ws)
     when (search description predicate :test 'equal)
       collect predicate))

(defun plot-by-predicate (sims pred &rest objects)
  "Plots each predicate on a separate chart, across simulators."
  (call-service "plot" 'plotter-srv:Plot
                :plots (loop for sim in sims collect 
                            (loop with states = (get-states-from-last-run sim)
                               with start-time = (time-of (first states))
                               with x = (make-array (length states) :adjustable t :fill-pointer 0)
                               with y = (make-array (length states) :adjustable t :fill-pointer 0)
                               for state in states
                               for predicates = (predicates-of state)
                               do ;(format t "~a~%" state) 
                                 (loop for p in predicates 
                                     when (match-predicate p pred objects)
                                     do (vector-push-extend (- (time-of state) start-time) x)
                                       (vector-push-extend (first (last p)) y))
                               finally (return (make-msg "plotter/PlotData"
                                                         (name) (format nil "Predicate (~a ~a) of ~a simulator"
                                                                        pred objects sim)
                                                         (x_label) "time"
                                                         (y_label) (format nil "(~a ~a)"
                                                                           pred objects)
                                                         (x_data) x
                                                         (y_data) y))))))

(defun expand-preds (pred-specs simulation)
  (loop for predicate in (loop for pred-spec in pred-specs 
                            append (get-matching-predicates pred-spec simulation))
     collect (subseq predicate 0 (- (length predicate) 1))))

(defun plot-by-simulator (sims preds &key (time-limit nil))
  "Plots each predicate on a separate chart, across simulations."
  (loop for sim in sims 
     do (call-service "plot" 'plotter-srv:Plot
                      :plots (loop for pred-spec in (expand-preds preds sim) collect 
                                  (loop ;with states = (get-states-from-last-run sim)
                                     with start-time = (time-of (first (reverse (find-instances 'world-state sim :all))))
                                     with states = (if time-limit
                                                       (reverse (find-instances 'world-state sim 
                                                                                (list '< 'time (+ start-time time-limit))))
                                                       (reverse (find-instances 'world-state sim :all)))
                                     with x = (make-array (length states) :adjustable t :fill-pointer 0)
                                     with y = (make-array (length states) :adjustable t :fill-pointer 0)
                                     for state in states
                                     for predicates = (predicates-of state)
                                     do ;(format t "~a~%" state) 
                                       (loop for pred in predicates 
                                          when (simple-match-predicate pred-spec pred)
                                          do (vector-push-extend (- (time-of state) start-time) x)
                                            (vector-push-extend ;(first (last p)) y))
                                             (extract-value pred) y))
                                     finally (return (make-msg "plotter/PlotData"
                                                               (name) (format nil "Predicate ~a of ~a simulator"
                                                                              pred-spec (instance-name-of (simulator-of sim)))
                                                               (x_label) "time"
                                                               (y_label) (format nil "~a"
                                                                                 pred-spec)
                                                               (x_data) x
                                                               (y_data) y)))))))