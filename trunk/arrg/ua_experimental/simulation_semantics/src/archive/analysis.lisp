(in-package :simulation_semantics)

;;============================================================
;; SAX Algorithm for converting a real-valued time series to sequence of symbols

;; breaks by the number of symbols
(defparameter *breaks-ht* 
  (let ((ht (make-hash-table)))
    (setf (gethash -1 ht) '(-0.001 0.001))
    (setf (gethash 2 ht) '(0))
    (setf (gethash 3 ht) '(-0.43 0.43))
    (setf (gethash 4 ht) '(-0.67 0 0.67))
    (setf (gethash 5 ht) '(-0.84 -0.25 0.25 0.84))
    (setf (gethash 6 ht) '(-0.97 -0.43 0.00 0.43 0.97))
    (setf (gethash 7 ht) '(-1.07 -0.57 -0.18 0.18 0.57 1.07))
    (setf (gethash 8 ht) '(-1.15 -0.67 -0.32 0.00 0.32 0.67 1.15))
    ht))

(defparameter *alphabet* 
  '("a" "b" "c" "d" "e" "f" "g" "h" "i" "j" "k" "l" "m"
    "n" "o" "p" "q" "r" "s" "t" "u" "v" "w" "x" "y" "z"))

(defun sliding-average (time-series window-size)
  (loop for i from 0 to (- (length time-series) window-size)
     collect (/ (apply '+ (subseq time-series i (+ i window-size)))
                window-size)))

(defun normalize-series (time-series)
  "Normalizes to standard normal distribution"
  (let* ((mean (mean time-series))
         (std-dev (standard-deviation time-series)))
    (if (zerop std-dev)
        (loop for value in time-series 
           collect (- value mean))
        (loop for value in time-series
           collect (/ (- value mean) std-dev)))))

(defun convert-to-symbols (time-series &key (alphabet-size 3))
  "Input sequence should be normalized"
  (let ((breaks (gethash alphabet-size *breaks-ht*)))
    (loop for v in time-series for index = 0
       if (equal v 'unknown) do (setf index v)
       else do
         (loop while (and (< index (length breaks))
                          (> v (nth index breaks))) 
            do (incf index))
         (setf index (- alphabet-size index))
       collect index)
    )
  )

(defun sax (time-series &key (alphabet-size 3))
  (let* ((normalized (normalize-series time-series)))
    (convert-to-symbols normalized :alphabet-size alphabet-size)))
  
(defun sax-string (sax-sequence)
  (apply 'concatenate 'string
         (loop for index in sax-sequence 
            collect (nth (- index 1) *alphabet*))))

(defun low-med-high (sax-sequence)
  (let* ((lmh '(high medium low)))
    (loop for index in sax-sequence
       if (eq index 'unknown) collect 'unknown
       else collect (nth (- index 1) lmh))))

(defun mean (values)
  (/ (apply '+ values) (length values)))

(defun standard-deviation (values)
  (if (< (length values) 2) (return-from standard-deviation 0))
  (let* ((x-bar (mean values))
         (n (length values)))
    (sqrt (* (/ 1 (- n 1)) 
             (loop for x in values summing (expt (- x x-bar) 2))))))
       
;; Shape Library

(defun delta-ts (column)
  (let ((results '()))
    (loop for i from 0 below (- (length column) 1)
       for v1 = (nth i column) for v2 = (nth (1+ i) column)
       if (or (equal v1 'unknown) (equal v2 'unknown))
       do (push 'unknown results)
       else do
         (push (- v2 v1) results))
    (setf results (reverse results))
    (push 'unknown results)
    results))

(defun shape-sdl (deltas)
  (loop for d in deltas collect
       (cond ((equal d 'unknown) d)
             ((< d -0.005) 'decreasing)
             ((> d 0.005) 'increasing)
             (t 'stable))))

;;================================================================
;; These were taken from time-series.lisp

#+ignore(defun symbolize-mts (mts)
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

#+ignore(defun symbolic-mts-to-pmts (mts)
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
       
#+ignore(defun make-proposition (predicate-name value)
  (format nil "~a=~a" predicate-name value))

#+ignore(defun make-propositions (predicate-name ts)
  (let* ((value-set (loop with values = (make-hash-table :test 'eq)
                       for value in ts
                       unless (or (eq value 'unknown) ;; Effectively treating unknown as negation
                                  #+ignore(null value))
                       do (setf (gethash value values) t)
                       finally (return values))))
    (loop for value being the hash-keys of value-set 
       collect (make-proposition predicate-name value))))

#+ignore(defun convert-predicate-to-key-value (predicate &key (name-map nil))
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


