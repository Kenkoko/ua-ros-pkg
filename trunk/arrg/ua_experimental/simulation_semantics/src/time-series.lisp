(in-package :simsem)

;;==========================================================================

(defun intervals-to-episode (intervals)
  (make-msg "time_series/Episode"
            (intervals) (ros-list (loop for interval in intervals
                                     collect (make-msg "time_series/Interval"
                                                       (proposition) (first interval)
                                                       (start) (second interval)
                                                       (end) (third interval))))))
  
(defun remap-arguments (proposition name-map)
  (loop with prop = proposition
     for specific being the hash-keys of name-map using (hash-value general)
     do (setf prop (replace-all prop specific (string general)))
     finally (return prop)))

(defun remap-name (name name-map)
  (if name-map 
      (let* ((arg-name (gethash name name-map)))
        (if arg-name arg-name name))
      name))

;;==========================================================================

;; TODO: Move these to archive in analysis.lisp

(defun pmts-to-intervals (pmts)
  ;(loop for pair in pmts do (print pair))
  (loop for pair in pmts
     append (extract-intervals pair)))

(defun extract-intervals (pair)
  (loop with interval-start = nil
     with series = (second pair)
     for i from 0
     for value in series
     if (and value (not interval-start))
     do (setf interval-start i)
     else if (and (not value) interval-start)
     collect (list (first pair) interval-start i) into interval-list
     and do (setf interval-start nil)
     finally (return (if interval-start 
                         (append interval-list 
                                 (list (list (first pair) interval-start i)))
                         interval-list))))
              
;;================================================================
;; New System

(defun convert-relation (rel-msg name-map)
  (let* ((rel-name (oomdp_msgs-msg:relation-val rel-msg))
         (obj-names (vec-to-list (oomdp_msgs-msg:obj_names-val rel-msg)))
         (value (oomdp_msgs-msg:value-val rel-msg)))
    (setf obj-names (loop for name in obj-names collect (remap-name name name-map)))
    (list (format nil "~a(~{~a~^,~})" rel-name obj-names) value)))

(defun convert-relations (state name-map)
  (loop for rel across (oomdp_msgs-msg:relations-val state)
     collect (convert-relation rel name-map)))

(defun print-traces (scenario name-map)
  (loop for trace in (traces-of scenario)
     for i from 0
     do (format t "Trace ~a:~%" i)
       (loop for state in trace
          for j from 0
          for relations = (convert-relations state name-map)
          do (format t "~tState ~a:~%" j)
            (loop for rel in relations do (format t "~t~a~%" rel)))
       (format t "~%")))

(defun get-relation-set (trace name-map)
  (loop with state = (first trace)
     for relation in (convert-relations state name-map)
     collect (first relation)))

;; NB: This will not work if relation set is changing, but that should never happen
;; TODO: Probably should signal an error if that happens
(defun convert-trace-to-pmts (trace name-map)
  (loop with relation-set = (get-relation-set trace name-map)
     with pmts-hash = (make-hash-table :test 'equal) 
       ;(loop with hash = 
       ;                  for relation in relation-set 
       ;                  do (setf (gethash (list relation nil))
     for state in trace
     do (loop for relation in (convert-relations state name-map)
           do (push (second relation) (gethash (first relation) pmts-hash)))
     finally (return (loop for relation in relation-set
                        collect (list relation (nreverse (gethash relation pmts-hash)))))))     
