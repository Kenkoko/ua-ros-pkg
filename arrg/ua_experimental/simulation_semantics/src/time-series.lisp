(in-package :simsem)

;;==========================================================================

(defun intervals-to-episode (intervals)
  (make-msg "time_series/Episode"
            (intervals) (ros-list (loop for interval in intervals
                                     collect (make-msg "time_series/Interval"
                                                       (proposition) (first interval)
                                                       (start) (second interval)
                                                       (end) (third interval))))))
  
;;==========================================================================

(defmethod convert-to-intervals ((simn simulation) &key (name-map nil))
  (pmts-to-intervals (symbolic-mts-to-pmts 
                      (symbolize-mts 
                       (world-states-to-mts (get-states simn)
                                            :name-map name-map)))))

(defun world-states-to-mts (raw-world-states &key (name-map nil))
  "Converts a list of world-state objects into a time series"
  ;; The first timestep is quirky, let's just kill it for now
  (let* ((world-states (rest raw-world-states))
         (predicate-set (make-hash-table :test 'equal))
         (ts-map (make-hash-table :test 'equal))
         (predicate-list nil))
    ;; Get the predicate names
    (loop for ws in world-states do
         (loop for predicate in (predicates-of ws)
            do (setf (gethash (first (convert-predicate-to-key-value predicate :name-map name-map)) predicate-set) t)))
    ;; Sort the predicates into a canonical order
    (setf predicate-list (loop for predicate being the hash-keys of predicate-set collect predicate))
    (sort predicate-list 'string-lessp)
    ;; Loop through all the states and build time-series
    (loop for ws in world-states
       do (loop for predicate in (predicates-of ws)
             for key-value = (convert-predicate-to-key-value predicate :name-map name-map)
             do (push (second key-value) (gethash (first key-value) ts-map))))
    (loop for predicate being the hash-keys of ts-map using (hash-value ts)
       collect (list predicate (reverse ts)))
))

(defun symbolize-mts (mts)
  "Converts a multivariate time series into a propositional MTS with SAX"
  ;(loop for pair in mts do (print pair))
  (loop for pair in mts
     for predicate = (first pair)
     for ts = (second pair)
     if (numberp (first ts))
     append (list (list predicate (low-med-high (sax ts)))
                  (list (format nil "Shape(~a)" predicate) (shape-sdl (delta-ts ts))))
     else 
     append (list (list predicate ts))))

(defun symbolic-mts-to-pmts (mts)
  ;(loop for pair in mts do (print pair))
  (let* ((propositions (loop for pair in mts append (make-propositions (first pair) (second pair))))
         (series-length (length (second (first mts))))
         (pmts nil))
    (loop for p in propositions do (setf pmts (append pmts (list (list p nil)))))
    (loop for i below series-length
       for true-props = (loop with true-set = (make-hash-table :test 'equal)
                           for pair in mts
                           for pred = (first pair)
                           for value = (nth i (second pair))
                           do (setf (gethash (make-proposition pred value) true-set) t)
                           finally (return true-set))
       do (loop for j below (length propositions)
             do (push (gethash (nth j propositions) true-props) (second (nth j pmts)))))
    (loop for pair in pmts do (setf (second pair) (reverse (second pair))))
    pmts))
       
(defun pmts-to-intervals (pmts)
  ;(loop for pair in pmts do (print pair))
  (loop for pair in pmts
     append (extract-intervals pair)))

(defun extract-intervals (pair)
  (loop with interval-list = nil
     with name = (first pair)
     with series = (second pair)
     with interval-start = nil
     for index below (length series)
     for value = (nth index series)
     if (eq value t) ;; To deal with unknowns
     do (if interval-start 
            (if (= index (- (length series) 1))
                (push (list name interval-start (length series)) interval-list))
            (setf interval-start index))
     else if interval-start
     do (push (list name interval-start index) interval-list)
       (setf interval-start nil)
     finally (return (reverse interval-list))))
              
(defun make-proposition (predicate-name value)
  (format nil "~a=~a" predicate-name value))

(defun make-propositions (predicate-name ts)
  (let* ((value-set (loop with values = (make-hash-table :test 'eq)
                       for value in ts
                       unless (or (eq value 'unknown) ;; Effectively treating unknown as negation
                                  #+ignore(null value))
                       do (setf (gethash value values) t)
                       finally (return values))))
    (loop for value being the hash-keys of value-set 
       collect (make-proposition predicate-name value))))

(defun convert-predicate-to-key-value (predicate &key (name-map nil))
  (let* ((entities (subseq predicate 1 (- (length predicate) 1)))
         (value (first (last predicate))))
    (if name-map
        (list (format nil "~a(~{~a~^,~})"(first predicate) 
                      (loop for name in entities 
                         for new-name = (gethash name name-map)
                         if new-name collect new-name
                         else collect name))
              value)
        (list (format nil "~a(~{~a~^,~})"(first predicate) entities)
              value))))

