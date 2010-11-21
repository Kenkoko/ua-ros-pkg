(in-package :simsem)

;;===========================================================
;; Found on the web
(defun copy-hash-table (table &key key test size
                                   rehash-size rehash-threshold)
  "Returns a copy of hash table TABLE, with the same keys and values
as the TABLE. The copy has the same properties as the original, unless
overridden by the keyword arguments.

Before each of the original values is set into the new hash-table, KEY
is invoked on the value. As KEY defaults to CL:IDENTITY, a shallow
copy is returned by default."
  (setf key (or key 'identity))
  (setf test (or test (hash-table-test table)))
  (setf size (or size (hash-table-size table)))
  (setf rehash-size (or rehash-size (hash-table-rehash-size table)))
  (setf rehash-threshold (or rehash-threshold (hash-table-rehash-threshold table)))
  (let ((copy (make-hash-table :test test :size size
                               :rehash-size rehash-size
                               :rehash-threshold rehash-threshold)))
    (maphash (lambda (k v)
               (setf (gethash k copy) (funcall key v)))
             table)
    copy))

;;===========================================================
;; Useful for dealing with ROS messages

(defun to-string-list (seq)
  (mapcar 'string-downcase seq))

(defun ros-list (seq)
  (make-array (length seq) :initial-contents seq))

(defun msg-equal (a-msg b-msg)
  "Like equal, but applies to ROS messages."
  (if (eq (class-of a-msg) (class-of b-msg))
      (cond ((is-instance-of a-msg 'ros-message)
             (loop for slot in (class-slots (class-of a-msg))
                when (not (msg-equal (slot-value a-msg (slot-definition-name slot))
                                     (slot-value b-msg (slot-definition-name slot))))
                do (return nil)
                finally (return t)))
             ((vectorp a-msg)
              (cl-utils:is-permutation (vec-to-list a-msg) (vec-to-list b-msg) :test 'msg-equal))
             (t
              (equal a-msg b-msg)))))
      
(defun vec-to-list (vec)
  (loop for x across vec collect x))

;;===========================================================
;; Printing

(defun default-print-slots (obj stream)
  (format stream " ~{~a~^, ~}"
          (loop with obj-class = (class-of obj)
             for slot-def in (class-direct-slots obj-class)
             for slot-name = (slot-definition-name slot-def)
             collect (format nil "~a: ~a" slot-name
                             (if (slot-boundp obj slot-name)
                                 (slot-value obj slot-name) 
                                 "<UNBOUND>")))))

(defun print-hash (hash)
  (loop for k being the hash-keys of hash using (hash-value v)
     do (format t "~a => ~a~%" k v)))

;;===========================================================
;; Classes and instances

(defun is-instance-of (object class-name)
  (find (find-class class-name) (class-precedence-list (class-of object))))

(defun fibn (name)
  "shortcut for (find-instance-by-name name)"
  (find-instance-by-name name))

;;===========================================================
;; Math

(defun radians-to-degrees (rad)
  (* rad (/ 180 pi)))

(defun mean (values)
  (/ (apply '+ values) (length values)))

(defun standard-deviation (values)
  (if (< (length values) 2) (return-from standard-deviation 0))
  (let* ((x-bar (mean values))
         (n (length values)))
    (sqrt (* (/ 1 (- n 1)) 
             (loop for x in values summing (expt (- x x-bar) 2))))))

(defun factorial (n &optional (acc 1))
  (if (<= n 1)
      acc
      (factorial (- n 1) (* acc n))))

;; n is the sequence length, k is the subsequence length
(defun permutations (n k)
  (/ (factorial n)
     (factorial (- n k))))

(defun unordered-permutations (n k)
  (if (= k 1)
      k
      (/ (permutations n k) 2)))

;; cl-utils has a version of this
(defun within-delta? (x y delta)
  "Returns true if x is within +/- delta of y."
  (and (< x (+ y delta)) (> x (- y delta))))

;;===========================================================

(defun boolean-string (value)
  (if value "true" "false"))

;;===========================================================

;; This is taken from the web: http://cl-cookbook.sourceforge.net/strings.html
(defun replace-all (string part replacement &key (test #'char=))
  "Returns a new string in which all the occurences of the part is replaced with replacement."
    (with-output-to-string (out)
      (loop with part-length = (length part)
            for old-pos = 0 then (+ pos part-length)
            for pos = (search part string
                              :start2 old-pos
                              :test test)
            do (write-string string out
                             :start old-pos
                             :end (or pos (length string)))
            when pos do (write-string replacement out)
            while pos))) 
