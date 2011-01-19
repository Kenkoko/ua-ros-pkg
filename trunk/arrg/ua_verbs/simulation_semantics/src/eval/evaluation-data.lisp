(in-package :simsem)

;;=============================================================================
;; Training

(defun make-training (domain verb)
  (cond ((eq domain 'gazebo)
         (get-gazebo-training verb))
        ((eq domain 'ww2d)
         (get-ww2d-training verb))))

;;=============================================================================
;; Testing

(defun make-tests (domain verb)
  (cond ((eq domain 'gazebo)
         (get-gazebo-tests verb))
        ((eq domain 'ww2d)
         (get-ww2d-tests verb))))

;;========================================================================
;; Utility functions

(defun test-examples (domain verb)
  (initialize (first (first (make-training domain verb)))))

(defun debug-training (instance)
  (let* ((start-state (initialize (first instance))))
    (print-relations start-state)
    (loop for action in (second instance)
       do (format t "Performing action ~a~%" action)
       do (let* ((new-state (perform-action action)))
            (format t "Action complete.~%")
            (print-relations new-state)))))
       
(defun print-relation-if-true (rel-msg)
  (let* ((rel-name (oomdp_msgs-msg:relation-val rel-msg))
         (obj-names (vec-to-list (oomdp_msgs-msg:obj_names-val rel-msg)))
         (value (oomdp_msgs-msg:value-val rel-msg)))
    (if value 
        (format t "~a(~{~a~^,~})~%" rel-name obj-names))))

(defun print-relations (state)
  (format t "~%>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>~%")
  (loop for relation across (oomdp_msgs-msg:relations-val state)
     do (print-relation-if-true relation)))