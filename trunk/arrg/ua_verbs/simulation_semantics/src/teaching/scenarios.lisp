(in-package :simsem)

;;=====================================================================
;; Teaching verbs and configuring scenarios

;;=====================================================================
;; Creating a new scenario

(defun new-scenario ()
  (killall)
  (setf *current-scenario* (make-instance 'simulator 
                                          :instance-name (gensym (lexical-form-of *current-verb*))))
  (format t "Empty scenario created.~%"))

;;=====================================================================
;; Adding entities to the scenario

;; Just one robot for now
;; angle is in degrees, left is positive (counter-clockwise rotation)
(defun add-robot (&key (position '(0 0 0)) (orientation 0))
  (if (null (fibn 'robot))
      (make-instance 'robot 
                     :instance-name 'robot
                     :gazebo-name "robot_description"))
  (let* ((pose (list position (list 0 0 (* orientation (/ pi 180)))))) 
    (setf (gethash (fibn 'robot) (objects-of *current-scenario*)) pose)
    (add-to-world (fibn 'robot) pose)
    (format t "Robot has been added to scenario at (~a,~a) with orientation ~a." 
            (first position) (second position) orientation)))

(defun add-object (&key (shape 'box) (static nil) (size 0.3) (name (gensym "OBJECT"))
                   (mass 0.2) (x 0) (y 0) (z nil) (orientation 0))
  (let* ((obj (make-instance 'physical-object
                             :instance-name name
                             :gazebo-name (string name)
                             :shape shape
                             :static? static
                             :size size
                             :mass mass))
         (where (if z
                    (list (list x y z) (list 0 0 (* orientation (/ pi 180))))
                    (list (list x y (/ (height-of obj) 2)) (list 0 0 (* orientation (/ pi 180)))))))
    (setf (gethash obj (objects-of *current-scenario*)) where)
    (add-to-world obj where)
    (format t "Object has been added to scenario at (~a,~a,~a)." (first (first where)) (second (first where)) (third (first where)))))

;; TODO: Add each to the world as you construct the scenario
;; Remove when scenario is complete
;; z is always 0 for now
(defun add-position-goal (target-xy)
  (let* ((goal (make-instance 'position-goal 
                              :entity-name "robot_description" ;(gazebo-name-of (fibn 'robot))
                              :position (make-instance 'xyz 
                                                       :x (first target-xy) 
                                                       :y (second target-xy) 
                                                       :z 0)))
         (pose (list (list (x-of (position-of goal)) (y-of (position-of goal)) 0.1) '(0 0 0))))
    (setf (goals-of *current-scenario*) (list goal))
    (if (null (fibn 'goal-marker))
        (make-instance 'physical-object
                       :instance-name 'goal-marker
                       :gazebo-name "goal_marker"
                       :shape 'sphere
                       :color "Gazebo/Green"
                       :size 0.1))
    (add-to-world (fibn 'goal-marker) pose)
    (add-to-world goal nil)
    (format t "Robot will try to achieve position (~a,~a).~%" (first target-xy) (second target-xy))))