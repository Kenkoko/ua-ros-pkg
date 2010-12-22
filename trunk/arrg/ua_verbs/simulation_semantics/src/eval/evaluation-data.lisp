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
