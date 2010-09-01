(in-package :simulation_semantics)

;;======================================================================
;; This file contains code for controlling the simple robot base
;;======================================================================

(define-unit-class robot (entity)
  ((vel-pub :initform nil)
   (intended-velocity :initform 0))
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

(defmethod add-to-world ((r robot) pose)
  (call-service "gazebo/spawn_gazebo_model" 'gazebo-srv:SpawnModel 
                :model_name "robot_description"
                :model_xml (xml-rep r)
                :initial_pose (make-initial-pose pose)
                :robot_namespace "/"))

(defmethod remove-from-world ((r robot))
  (call-service "gazebo/delete_model" 'gazebo-srv:DeleteModel
                :model_name (model-name r)))

(defmethod xml-rep ((r robot))
  (print-xml-string *robot-xml* :pretty t))

(defmethod get-default-predicates ((r robot))
  '(x-pos y-pos z-pos
    moving turning-left turning-right 
    ;x-vel y-vel z-vel 
    vel-mag ;diff-speed
    ;force-mag
    yaw pitch roll))
    ;yaw-vel pitch-vel roll-vel))

;;=====================================================================

;; TODO: Hardcoded for now
(defmethod model-name ((r robot))
  "robot_description")

(defmethod move-robot ((r robot) forward-vel rot-vel)
  ;(format t "~a ~a~%" forward-vel rot-vel)
  (setf (intended-velocity-of r) (list forward-vel rot-vel))
  (publish (vel-pub-of r) (make-msg "geometry_msgs/Twist"
                                    (x linear) forward-vel
                                    (z angular) rot-vel)))

;;=====================================================================

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
               (move-robot r (first vel) (second vel))))))
    
      