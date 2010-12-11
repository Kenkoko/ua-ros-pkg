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

(defun argument-names (object-states)
  (loop for object-state across object-states
     collect (oomdp_msgs-msg:name-val object-state)))           

;; NB: Does not preserve ordering!
(defun sample-random-subset (seq size)
  (loop with remaining = seq
     for i below size
     for chosen = (nth (random (length remaining)) remaining)
     do (setf remaining (remove chosen remaining))
     collect chosen))

(defun test-examples (domain verb)
  (initialize (first (first (make-training domain verb)))))