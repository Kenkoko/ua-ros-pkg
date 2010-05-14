(in-package :simulation_semantics)

;;======================================================================
;; This file contains code for controlling the simple robot base
;;======================================================================

(define-unit-class robot (thing)
  ((vel-pub :initform nil)
   )
  )

;;=====================================================================
;; Methods

(defmethod activate ((r robot))
  (if (vel-pub-of r)
      (ros-warn "robot.lisp" "Robot was already activated?"))
  (setf (vel-pub-of r) (advertise "cmd_vel" "geometry_msgs/Twist" :latch nil)))

(defmethod deactivate ((r robot))
  (unadvertise "cmd_vel")
  (setf (vel-pub-of r) nil))

;; NOTE: This is exactly the same in physical-object, can we move it up?
(defmethod add-to-world ((r robot))
  (call-service "add_model" 'gazebo_plugins-srv:SpawnModel 
                :model (make-message "gazebo_plugins/GazeboModel" 
                                     :xml_type 1
                                     :robot_model (xml-rep r))))

;; TODO: Model name should not be hardcoded
(defmethod remove-from-world ((r robot))
  (call-service "delete_model" 'gazebo_plugins-srv:DeleteModel
                :model_name "robot_description"))

;; TODO: Need to use the robot's location not just what's in the XML
(defmethod xml-rep ((r robot))
  (print-xml-string *robot-xml* :pretty t))

;;=====================================================================

(defmethod move-robot ((r robot) forward-vel rot-vel)
  (format t "~a ~a~%" forward-vel rot-vel)
  (publish (vel-pub-of r) (make-msg "geometry_msgs/Twist"
                                    (x linear) forward-vel
                                    (z angular) rot-vel)))

;; TODO: Move these somewhere else
(defun within-delta? (x y delta)
  "Returns true if x is within +/- delta of y."
  (and (< x (+ y delta)) (> x (- y delta))))

(defun demo-policy (r sim-time ws)
  (declare (ignore ws))
  (let* ((vel (cond ((< sim-time 1.0)
                     '(0 0))
                    ((within-delta? sim-time 1.0 0.01)
                     '(0.5 0))
                    ((within-delta? sim-time 8.0 0.01)
                     '(0 0))
                    ;((within-delta? sim-time 10.5 0.01)
                    ; '(-0.5 0 0))
                    ((within-delta? sim-time 9.0 0.01)
                     '(0 0.5))
                    ((within-delta? sim-time 10.0 0.01)
                     '(0 0))
                    ((within-delta? sim-time 10.5 0.01)
                     '(0.5 0))
                    ((within-delta? sim-time 17.0 0.01)
                     '(0 0))
                    (t nil))))
    (if vel
        (progn (format t "TIME: ~a, VEL: ~a~%" sim-time vel)
               (funcall 'move-robot r (first vel) (second vel))))))
    
      