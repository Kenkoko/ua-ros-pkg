(in-package :simulation_semantics)

;;======================================================================
;; This file contains code for controlling the simple robot base
;;======================================================================

(defparameter *robot-pub* nil)

(defun init-robot ()
  (setf *robot-pub* (advertise "cmd_vel" "geometry_msgs/Twist" :latch nil)))

(defun cleanup-robot ()
  (unadvertise "cmd_vel"))

(defun move-robot (x-vel y-vel rot-vel)
  ;(declare (ignore robot-name))
  (format t "~a ~a ~a~%" x-vel y-vel rot-vel)
  (publish *robot-pub* (make-msg "geometry_msgs/Twist"
                                 (x linear) x-vel
                                 (y linear) y-vel
                                 (z angular) rot-vel)))

(defun within-delta? (x y delta)
  "Returns true if x is within +/- delta of y."
  (and (< x (+ y delta)) (> x (- y delta))))

(defun demo-policy (sim-time ws)
  (declare (ignore ws))
  (let* ((vel (cond ((< sim-time 0.5)
                     '(0 0 0))
                    ((within-delta? sim-time 0.5 0.01)
                     '(0.5 0 0))
                    ((within-delta? sim-time 13.0 0.01)
                     '(0 0 0))
                    ((within-delta? sim-time 14.5 0.01)
                     '(-0.5 0 0))
                    ((within-delta? sim-time 15.0 0.01)
                     '(0 0 0.5))
                    ((within-delta? sim-time 17.5 0.01)
                     '(0 0 0))
                    ((within-delta? sim-time 19.0 0.01)
                     '(0.5 0 0))
                    ((within-delta? sim-time 24.0 0.01)
                     '(0 0 0))
                    (t nil))))
    (if vel
        (progn (format t "TIME: ~a, VEL: ~a~%" sim-time vel)
               (funcall 'move-robot (first vel) (second vel) (third vel))))))
    
      