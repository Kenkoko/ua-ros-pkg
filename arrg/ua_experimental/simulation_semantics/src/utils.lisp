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

;;==========================================================================

(defun print-hash (hash)
  (loop for k being the hash-keys of hash using (hash-value v)
     do (format t "~a => ~a~%" k v)))

;;===========================================================

(defun ros-list (seq)
  (make-array (length seq) :initial-contents seq))

;;===========================================================
;; Utility method for printing simple objects

(defun default-print-slots (obj stream)
  (format stream " ~{~a~^, ~}"
          (loop with obj-class = (class-of obj)
             for slot-def in (class-direct-slots obj-class)
             for slot-name = (slot-definition-name slot-def)
             collect (format nil "~a: ~a" slot-name
                             (if (slot-boundp obj slot-name)
                                 (slot-value obj slot-name) 
                                 "<UNBOUND>")))))

;;===========================================================

(defun within-delta? (x y delta)
  "Returns true if x is within +/- delta of y."
  (and (< x (+ y delta)) (> x (- y delta))))

;;===========================================================

(defun is-instance-of (object class-name)
  (find (find-unit-class class-name) (class-precedence-list (class-of object))))

;;===========================================================

(defun radians-to-degrees (rad)
  (* rad (/ 180 pi)))

;;===========================================================

(defun vec-to-list (vec)
  (loop for x across vec collect x))

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
