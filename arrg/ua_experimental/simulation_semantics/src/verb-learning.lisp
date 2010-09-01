(in-package :simsem)

;;=====================================================================
;; Verb Class

(define-unit-class verb () 
  (lexical-form
   argument-structure
   (scenarios :initform nil)
   (signature :initform nil)
   (interventions :initform nil))
)

(define-unit-class scenario ()
  ((argument-list :initform nil)
   (simulator :initform nil))
)

(define-unit-class counterfactual-scenario ()
  (scenario
   intervention
   original-scenario))
   
;;=====================================================================

(defparameter *current-verb* nil)
(defparameter *world-state-subscriber* nil)
(defparameter *current-scenario* nil)
(defparameter *current-cf-scenario* nil)
(defparameter *current-arguments* nil)

(defun begin-verb-lesson (verb arguments)
  (format t "Lesson begun for verb: ~a(~{~a~^,~})~%" verb arguments)
  (setf *current-verb* (make-instance 'verb :lexical-form verb :argument-structure arguments)))

(defun present-scenario (simulator arguments)
  "Arguments is a list that has the object names in the appropriate argument order"
  (push (make-instance 'scenario :argument-list arguments :simulator simulator)
        (scenarios-of *current-verb*)))
  
(defun simulate-scenarios (&key (verb *current-verb*))
  (unless *world-state-subscriber*
    (setf *world-state-subscriber* (subscribe-to-world-state)))
  (loop for scenario in (scenarios-of verb)
     do (run-simulator (simulator-of scenario))))
                        
;;======================================================================
;; Signatures

(defun learn-canonical-signature (&key (verb *current-verb*))
  (call-service "find_signature" 'time_series-srv:FindSignature
                :episodes (get-episodes verb)
                :verb (lexical-form-of verb)))

(defun learn-counterfactual-signature (&key (verb *current-verb*))
  (call-service "find_signature" 'time_series-srv:FindSignature
                :episodes (get-intervention-episodes verb)
                :verb (concatenate 'string (lexical-form-of verb) "-counterfactual")))
 
(defun test-scenario (&key (scenario *current-scenario*) (verb *current-verb*))
  ;(clear-preview)
  (format t "Simulating scenario...~%")
  (let* ((simn (run-simulator scenario))
         (verb-arguments (argument-structure-of verb))
         (possibles (possible-arguments))
         (name-map (loop with result = (make-hash-table :test 'equal)
                      for verb-arg in verb-arguments
                      do (format t "Which is the ~a?~%" (string-capitalize (string verb-arg)))
                        (list-possible-arguments)
                        (let* ((index (- (read) 1))
                               (object (nth index possibles)))
                          (if (is-instance-of object 'entity)
                              (setf (gethash (gazebo-name-of object) result) verb-arg)))
                      finally (return result)))
         (episode (simulation-to-episode simn name-map)))
    (format t "Testing episode against canonical signature of ~a~%" (lexical-form-of verb))
    (format t "Distance: ~a~%" (score-val (test-episode episode (lexical-form-of verb) :min 2)))
    (format t "Testing episode against intervention signature of ~a~%" (lexical-form-of verb))
    (let* ((cf (duplicate scenario (gensym "CFS")))
           (fake-map (make-hash-table :test 'equal)))
      (setf (objects-of cf) (make-hash-table :test 'equal))
      (setf (gethash (fibn 'robot) (objects-of cf)) (gethash (fibn 'robot) (objects-of scenario)))
      (setf (gethash "robot_description" fake-map) "robot_description")
      (let* ((cf-simn (run-simulator cf))
             (cf-episode (other-simulation-to-episode cf-simn)))
        ;(print cf-episode)
        (format t "Distance: ~a~%" (score-val (test-episode cf-episode (concatenate 'string
                                                                                    (lexical-form-of verb)
                                                                                    "-counterfactual")
                                                            :min 2)))))))

(defun test-episode (episode verb &key (min 1))
  (call-service "test_signature" 'time_series-srv:TestSignature
                :episode episode
                :verb verb
                :min min))
                
(defun get-episodes (verb)
  (loop for scenario in (scenarios-of verb)
     append (loop for simulation in (simulations-of (simulator-of scenario))
               for episode = (simulation-to-episode simulation (name-map-of scenario verb))
               unless (instance-deleted-p simulation) ;; Hacky
               collect episode)))

(defun get-intervention-episodes (verb)
  (loop for intervention in (interventions-of verb)
     for scenario = (scenario-of intervention)
     append (loop for simulation in (simulations-of (simulator-of scenario))
               for episode = (simulation-to-episode simulation (name-map-of scenario verb))
               unless (instance-deleted-p simulation) ;; Hacky
               collect episode)))

(defun other-simulation-to-episode (simulation)
  (intervals-to-episode 
   (pmts-to-intervals 
    (symbolic-mts-to-pmts 
     (symbolize-mts 
      (world-states-to-mts 
       (get-states simulation)))))))

(defun simulation-to-episode (simulation name-map)
  (intervals-to-episode 
   (pmts-to-intervals 
    (symbolic-mts-to-pmts 
     (symbolize-mts 
      (world-states-to-mts 
       (get-states simulation) :name-map name-map))))))

;;======================================================================
;; Scenario Configuration

(defun new-scenario ()
  (setf *current-scenario* (make-instance 'simulator 
                                          :instance-name (gensym (lexical-form-of *current-verb*))))
  (format t "Empty scenario created.~%"))
                                          
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
                   (mass 1.0) (x 0) (y 0) (z nil) (orientation 0))
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
    (format t "Robot will try to achieve position (~a,~a).~%" (first target-xy) (second target-xy))))

;; Return the strings or print them?
(defun remove-position-goal ()
  (if (goals-of *current-scenario*)
      (progn (setf (goals-of *current-scenario*) nil)
             (remove-from-world (fibn 'goal-marker))))
  (format t "Position Goal Removed"))

(defun finish-scenario ()
  (let* ((verb-arguments (argument-structure-of *current-verb*))
         (possibles (possible-arguments))
         (argument-list (loop for verb-arg in verb-arguments
                           do (format t "Which is the ~a?~%" (string-capitalize (string verb-arg)))
                             (list-possible-arguments)
                           collect (nth (- (read) 1) possibles))))
    ;(print argument-list)))
    (setf *current-arguments* argument-list)
    (present-scenario *current-scenario* *current-arguments*)
    (destroy *current-scenario*)
    (remove-from-world (fibn 'goal-marker))
    (format t "This scenario described as: ~a(~{~a~^,~})~%" (lexical-form-of *current-verb*) *current-arguments*)))

;; TODO: Print something informative here
(defun scenario-complete ()
  (if *current-arguments*
      (present-scenario *current-scenario* *current-arguments*)
      (format t "Please specify the argument mapping!~%")))

;;======================================================================
;; Interventions

(defun new-global-intervention ()
  (let* ((interventions '(move-object delete-object)))
    (format t "Select an intervention:~%")
    (loop for int in interventions
       for i from 1
       do (format t "~a: ~a~%" i int))
    (let* ((index (read))
           (intervention (nth (- index 1) interventions)))
      (format t "Select the object:~%")
      (loop for arg in (argument-structure-of *current-verb*)
         for i from 1
         do (format t "~a: ~a~%" i arg))
      (let* ((object-index (- (read) 1))
             (cf-scenarios (loop for scenario in (scenarios-of *current-verb*)
                              for base-simulator = (simulator-of scenario)
                              collect (duplicate base-simulator (gensym "CFS")))))
        (loop for cf in cf-scenarios 
           for robot-pos = (gethash (fibn 'robot) (objects-of cf))
           do (setf (objects-of cf) (make-hash-table :test 'equal))
             (setf (gethash (fibn 'robot) (objects-of cf)) robot-pos)) ; SUPER HACK BAD BAD BAD
        (setf (interventions-of *current-verb*) 
              (loop for cf in cf-scenarios 
                 for i from 0
                 collect (make-instance 'counterfactual-scenario
                                        :original-scenario (nth i (scenarios-of *current-verb*))
                                        :intervention intervention
                                        :scenario (make-instance 'scenario
                                                                 :simulator cf
                                                                 :argument-list (list "robot_description"))))))
  (format t "Intervention has been applied to all scenarios, resulting in ~a counterfactual scenarios.~%"
          (length (interventions-of *current-verb*))))))

(defun new-intervention ()
  (format t "Select the base scenario:~%")
  (loop for scenario in (scenarios-of *current-verb*)
     for i from 1
     do (format t "~a: ~a~%" i scenario))
  (let* ((base-scenario (nth (- (read) 1) (scenarios-of *current-verb*)))
         (base-simulator (simulator-of base-scenario))
         (interventions '(move-object change-goal)))
    (preview-scenario :scenario base-simulator)
    (format t "Select an intervention:~%")
    (loop for int in interventions
       for i from 1
       do (format t "~a: ~a~%" i int))
    (let* ((index (read))
           (intervention (nth (- index 1) interventions)))
      (cond ((eq intervention 'move-object)
             (format t "Select the object:~%")
             (let* ((object-list (loop for obj being the hash-keys of (objects-of base-simulator)
                                    for i from 1 
                                    do (format t "~a: ~a~%" i obj)
                                    collect obj))
                    (object (nth (- (read) 1) object-list)))
               (format t "Where? (x y z)~%")
               (let* ((position (read))
                      (new-simulator (duplicate base-simulator (gensym "CFS"))))
                 (setf (gethash object (objects-of new-simulator)) (list position '(0 0 0)))
                 (push (make-instance 'counterfactual-scenario
                                      :original-scenario base-scenario
                                      :intervention (list intervention position)
                                      :scenario (make-instance 'scenario
                                                               :argument-list (argument-list-of base-scenario)
                                                               :simulator new-simulator))
                       (interventions-of *current-verb*)))))
            ((eq intervention 'change-goal)
             (format t "Where? (x y)~%")
             (let* ((position (read))
                    (new-simulator (duplicate base-simulator (gensym "CFS"))))
               (setf (goals-of new-simulator) 
                     (list (make-instance 'position-goal 
                                          :entity_name "robot_description"
                                          :position (make-instance 'xyz
                                                                   :x (first position)
                                                                   :y (second position)))))
               (push (make-instance 'counterfactual-scenario
                                    :original-scenario base-scenario
                                    :intervention (list intervention position)
                                    :scenario (make-instance 'scenario
                                                             :argument-list (argument-list-of base-scenario)
                                                             :simulator new-simulator))
                     (interventions-of *current-verb*))))
            (t (format t "WTF~%"))))))
          
(defun simulate-counterfactual-scenarios ()
  (loop for cfs in (interventions-of *current-verb*)
     do (run-simulator (simulator-of (scenario-of cfs)))))

;; TODO: Make a function for select from list by ID

;;======================================================================
;; Utils

(defmethod name-map-of ((s scenario) (v verb))
  (loop with map = (make-hash-table :test 'equal)
     for obj in (argument-list-of s)
     for arg in (argument-structure-of v)
     if (is-instance-of obj 'entity) ;; For now, we are just ignoring the goals since their names don't matter
     do (setf (gethash (gazebo-name-of obj) map) arg)
     finally (return map)))

(defun forget-simulations ()
  (delete-all-instances 'simulation))

(defun preview-all-scenarios (&key (verb *current-verb*))
  (loop for scenario in (scenarios-of verb)
     do (preview-scenario :scenario (simulator-of scenario))))

(defun preview-scenario (&key (scenario *current-scenario*))
  (format t "Previewing scenario...~%")
  (pause-simulation)
  (construct scenario)
  (let* ((goals (goals-of scenario))
         (marker (fibn 'goal-marker))
         (marker-added nil))
    (if (null marker)
        (make-instance 'physical-object
                       :instance-name 'goal-marker
                       :gazebo-name "goal_marker"
                       :shape 'sphere
                       :color "Gazebo/Green"
                       :size 0.1))
    (if goals (progn (add-to-world (fibn 'goal-marker)
                                   (list (list (x-of (position-of (first goals)))
                                               (y-of (position-of (first goals)))
                                               0.1)
                                         '(0 0 0)))
                     (setf marker-added t)))
    (read)
    (destroy scenario)
    (if marker-added (remove-from-world (fibn 'goal-marker)))))

(defun possible-arguments ()
  (append (loop for object being the hash-keys of (objects-of *current-scenario*) collect object)
          (goals-of *current-scenario*)))

(defun list-possible-arguments ()
  (loop for arg in (possible-arguments)
     for i from 1
     do (format t "~a: ~a~%" i arg)))

(defun reset-scenario (&key (scenario *current-scenario*))
  (destroy scenario)
  (remove-from-world (fibn 'goal-marker))
  (setf (objects-of scenario) (make-hash-table))
  (setf (goals-of scenario) nil))
  

(defun clear-preview ()
  (destroy *current-scenario*)
  (if (fibn 'goal-marker)
      (remove-from-world (fibn 'goal-marker))))


;; TODO: list-scenarios, preview-scenario, print-scenario

;(defun test-script ()
;  (delete-blackboard-repository)
;  ;(init-session)
;  (setf sim (make-instance 'simulator :instance-name 'block))
;  (setf block (make-instance 'physical-object :gazebo-name "test_box"))
  
;  (setf (gethash block (objects-of sim)) (list '(0 0 1.0) '(0 0 0)))
;  (begin-verb-lesson "stay" '("object"))
;  (present-scenario sim '("test_box"))
;  (simulate-scenarios *current-verb*))