(in-package :simsem)

(define-unit-class goal ()
  (entity-name) ;; Let's just use the gazebo-name I guess
)

(define-unit-class position-goal (goal)
  (position) ;; Let's make this an xyz
)

(define-unit-class object-goal (goal)
  (object-name)
)

;;==========================================================

(defgeneric get-target-position (goal world-state)
  (:documentation "Get the position the goal currently corresponds to"))

(defmethod get-target-position ((g position-goal) (ws world-state))
  (declare (ignore ws))
  (position-of g))

(defmethod get-target-position ((g object-goal) (ws world-state))
  (position-of (pose-of (get-object-state ws (find-object-by-gazebo-name (object-name-of g))))))

;;==========================================================

(defgeneric distance-to-goal (goal world-state)
  (:documentation "Compute the distance to the goal based on the world-state and its predicates"))

(defmethod distance-to-goal ((g position-goal) (ws world-state))
  (let* ((obj-state (get-object-state ws (find-object-by-gazebo-name (entity-name-of g))))
         (obj-position (position-of (pose-of obj-state)))
         (distance-to-goal (distance obj-position (position-of g))))
    (list (list "Distance-To-Goal" (entity-name-of g) distance-to-goal)
      (list "Goal-Reached" (entity-name-of g) (< distance-to-goal 0.2)))))

;; TODO: Use nconc so we can return lists, then return a Goal-Reached predicate also
(defmethod distance-to-goal ((g object-goal) (ws world-state))
  (let* ((distance-pred (match-predicates-in-state (list "GJK-Distance" 
                                                         (entity-name-of g)
                                                         (object-name-of g)) 
                                                   ws)))
    (list (list "Distance-To-Goal" (entity-name-of g) (extract-value distance-pred)))))

;;===========================================================

(defun new-seek-goal-policy (goal ws)
  (if ws
      (let* ((obj (find-object-by-gazebo-name (entity-name-of goal)))
             (robot-pos (position-of (pose-of (get-object-state ws obj))))
             (target-position (get-target-position goal ws))
             (angle (point-at-natural (x-of robot-pos) (y-of robot-pos)
                                      (x-of target-position) (y-of target-position)))
             (yaw (get-yaw (orientation-of (pose-of (get-object-state ws obj)))))
             (angular-vel (* 0.1 (- angle yaw))))
        (format t "DISTANCE: ~,3f ANGLE: ~,3f YAW: ~,3f VEL: ~,3f~%" 
                (distance robot-pos target-position) angle yaw angular-vel)
        (if (< (distance robot-pos target-position) 0.2)
            (move-robot obj 0.0 0.0)
            (move-robot obj 0.4 angular-vel)))))