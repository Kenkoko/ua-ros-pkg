(in-package :simsem)

;;=============================================================================
;; Training

;; REMEMBER TO TURN OFF PADDING LATER
(defun make-training (domain verb)
   (cond ((eq domain 'gazebo)
          ;(pad-training
          ; (get-gazebo-training verb)))
          (get-gazebo-training verb))
         ((eq domain 'ww2d)
          (get-ww2d-training verb))))

(defun pad-training (training)
  (loop with limit = 15
     for instance in training
     for actions = (second instance)
     for padded-actions = (append actions (loop for i below (- limit (length actions)) collect "drop"))
     collect (list (first instance) padded-actions)))
                                            

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