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

(defun argument-strings (verb)
  (mapcar 'string-downcase (argument-structure-of verb)))

(define-unit-class scenario ()
  ((argument-list :initform nil)
   (simulator :initform nil))
)

(define-unit-class counterfactual-scenario ()
  (scenario
   intervention
   original-scenario))
   
;;=====================================================================
;; Verbs

(defparameter *current-verb* nil)
(defparameter *world-state-subscriber* nil)
(defparameter *current-scenario* nil)
(defparameter *current-cf-scenario* nil)
(defparameter *current-arguments* nil)

(defun begin-verb-lesson (verb arguments)
  (format t "Lesson begun for verb: ~a(~{~a~^,~})~%" verb arguments)
  (setf *current-verb* (make-instance 'verb
                                      :instance-name (format nil "~a~a" verb "-verb")
                                      :lexical-form verb
                                      :argument-structure arguments)))

#+ignore(defun specialize-verb (verb parent new-arguments)
  (let* ((parent-verb (find-verb parent)))
    (if (not parent-verb)
        (format "PARENT DOES NOT EXIST~%"))
    (setf *current-verb* (make-instance 'verb
                                        :instance-name (format nil "~a~a" verb "-verb")
                                        :lexical-form verb
                                        :argument-structure new-arguments))
    (call-service "verb_learning/specialize_verb" 'verb_learning-srv:SpecializeVerb
                  :parent (make-verb-description parent-verb)
                  :new_verb (make-verb-description *current-verb*))))
;; TODO: Implement specialize on the other end.

(defun make-verb-description (verb)
  (make-message "verb_learning/VerbDescription"
                :verb (lexical-form-of verb)
                :arguments (ros-list (argument-strings verb))))
                
(defun present-scenario (simulator arguments)
  "Arguments is a list that has the object names in the appropriate argument order"
  (push (make-instance 'scenario :argument-list arguments :simulator simulator)
        (scenarios-of *current-verb*)))
  
(defun simulate-scenarios (&key (verb *current-verb*))
  (loop for scenario in (scenarios-of verb)
     do (run-simulator (simulator-of scenario))))
        
(defun find-verb (word)
  (loop for verb in (find-instances-of-class 'verb)
     if (string-equal (lexical-form-of verb) word)
     do (return verb)))
       
(defun blast ()
  (delete-all-instances 'verb 'simulator 'scenario))
         

(defun constrain-verb ()
  (call-service "verb_learning/constrain_verb" 'verb_learning-srv:ConstrainVerb
                :verb "go-around"
                :banned_props (list "Contact(thing,obstacle)")))

;;======================================================================
;; Signatures

(defun learn-canonical-signature (&key (verb *current-verb*))
  (call-service "verb_learning/find_signature" 'verb_learning-srv:FindSignature
                :episodes (get-episodes verb)
                :verb (make-message "verb_learning/VerbDescription"
                                    :verb (lexical-form-of verb)
                                    :arguments (ros-list (argument-strings verb)))))

(defun update-canonical-signature (episode &key (verb *current-verb*))
  (call-service "verb_learning/update_signature" 'verb_learning-srv:FindSignature
                :episodes (list episode)
                :verb (make-message "verb_learning/VerbDescription"
                                    :verb (lexical-form-of verb)
                                    :arguments (ros-list (argument-strings verb)))))

;; TODO: Fix this function
#+ignore(defun learn-counterfactual-signature (&key (verb *current-verb*))
  (call-service "verb_learning/find_signature" 'time_series-srv:FindSignature
                :episodes (get-intervention-episodes verb) 
                :verb (concatenate 'string (lexical-form-of verb) "-counterfactual")))

(defun load-verbs ()
  ;; TODO: Why does this crash?
  (call-service "verb_learning/load_verbs" 'verb_learning-srv:LoadVerbs))


;;=====================================================================
;; State Machines

(defun fsm-test (&key (scenario *current-scenario*) (verb *current-verb*))
  (let* ((arg-list (get-arguments scenario :verb verb))
         (name-map (loop with result = (make-hash-table :test 'equal)
                      for general in (argument-structure-of verb)
                      for specific in arg-list
                      do (setf (gethash (gazebo-name-of specific) result) (string-downcase (string general)))
                      finally (return result)))
         (callback (lambda (state)
                     (let* ((result (call-service "verb_learning/fsm_update" 'verb_learning-srv:FSMUpdate
                                                  :verb (lexical-form-of verb)
                                                  :relations (loop for relation in (convert-relations state name-map)
                                                                when (second relation)
                                                                collect (first relation)))))
                       (format t "RESULT:~%~a~%" result)))))
    (run-simulator scenario :state-callback callback)))

;;=====================================================================
;; Episodes

;; NEW

(defun get-episodes (verb)
  (loop for scenario in (scenarios-of verb)
     append (loop for trace in (traces-of (simulator-of scenario))
               for episode = (intervals-to-episode 
                              (pmts-to-intervals 
                               (convert-trace-to-pmts trace (name-map-of scenario verb))))
               collect episode)))

;;======================================================================
;; Scenario Configuration

(defun new-scenario ()
  (killall)
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

;; Return the strings or print them?
(defun remove-position-goal ()
  (if (goals-of *current-scenario*)
      (progn (setf (goals-of *current-scenario*) nil)
             (remove-from-world (fibn 'goal-marker))))
  (format t "Position Goal Removed"))

(defun finish-scenario ()
  (let* ((argument-list (get-arguments *current-scenario*)))
    (setf *current-arguments* argument-list)
    (present-scenario *current-scenario* *current-arguments*)
    (destroy *current-scenario*)
    (remove-from-world (fibn 'goal-marker))
    (format t "This scenario described as: ~a(~{~a~^,~})~%" (lexical-form-of *current-verb*) *current-arguments*)))



;; TODO: This is for the environment file
(defun killall () 
  (loop for model-name across (gazebo-srv:model_names-val 
                               (call-service "gazebo/get_world_properties" 'gazebo-srv:GetWorldProperties))
     unless (member model-name '("clock" "gplane" "point_white") :test 'string-equal)
     do (call-service "gazebo/delete_model" 'gazebo-srv:DeleteModel
                      :model_name model-name)))



;; WHOOMP THERE IT IS!
(defun perform-verb (&key (verb *current-verb*))
  (loop for scenario in (scenarios-of verb)
     for i from 1
     do (format t "Scenario ~d:~%" i)
       (loop for object in (argument-list-of scenario)
          for arg in (argument-structure-of verb)
          do (format t "~a => ~a at ~a~%" 
                     arg 
                     (class-name (class-of object))
                     (if (gethash object (objects-of (simulator-of scenario)))
                         (gethash object (objects-of (simulator-of scenario)))
                         (position-of (first (goals-of (simulator-of scenario))))))))
  (let* ((scenario (nth (- (read) 1) (scenarios-of verb))))
    (killall)
    (construct (simulator-of scenario))
    (let* ((start-state (oomdp_msgs-srv:start_state-val 
                         (call-service "environment/initialize" 'oomdp_msgs-srv:InitializeEnvironment
                                       :object_states #())))
           (plan (verb_learning-srv:plan-val 
                  (call-service "verb_learning/plan_verb" 'verb_learning-srv:PlanVerb
                                :verb (make-message "verb_learning/VerbInstance"
                                                    :verb (lexical-form-of verb)
                                                    :arguments (ros-list (argument-strings verb))
                                                    :bindings (ros-list (loop for object in (argument-list-of scenario)
                                                                           collect (gazebo-name-of object))))
                                :start_state start-state)))
           ;(states (verb_learning-msg:states-val plan))
           (actions (verb_learning-msg:actions-val plan))
           (new-trace (loop for action across actions 
                         do (format t "Performing action: ~a~%" action)
                         collect (oomdp_msgs-srv:new_state-val 
                                  (call-service "environment/perform_action" 'oomdp_msgs-srv:PerformAction
                                                :action action)) into trace
                         finally (return (push start-state trace))))
           (new-pmts (convert-trace-to-pmts new-trace (name-map-of scenario verb)))
           (new-episode (intervals-to-episode (pmts-to-intervals new-pmts))))
      (format t "Add episode to verb signature?~%")
      (if (read)
          (update-canonical-signature new-episode :verb verb)))))
           
;; TODO: Need to check states and replan if necessary

  






;; TODO: Need to consolidate the argument code
(defun get-arguments (sim &key (verb *current-verb*))
  (loop with verb-arguments = (argument-structure-of verb)
     with possibles = (possible-arguments :scenario sim)
     for verb-arg in verb-arguments
     do (format t "Which is the ~a?~%" (string-capitalize (string verb-arg)))
       (list-possible-arguments :scenario sim)
     collect (nth (- (read) 1) possibles)))

;; TODO: Print something informative here
(defun scenario-complete ()
  (if *current-arguments*
      (present-scenario *current-scenario* *current-arguments*)
      (format t "Please specify the argument mapping!~%")))

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
  (loop for verb in (find-instances-of-class 'verb)
     do (loop for scenario in (scenarios-of verb)
           do (setf (traces-of (simulator-of scenario)) nil))))

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

(defun possible-arguments (&key (scenario *current-scenario*))
  (append (loop for object being the hash-keys of (objects-of scenario) collect object)
          (goals-of scenario)))

(defun list-possible-arguments (&key (scenario *current-scenario*))
  (loop for arg in (possible-arguments :scenario scenario)
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

;;==============================================================

(defun clear-scenarios ()
  (setf (scenarios-of *current-verb*) nil))

(defun visualize-scenario ()
  ;(clear-preview)
  (run-simulator *current-scenario*))

(defun lookup-verb (lexical-form)
  (loop for verb in (find-instances-of-class 'verb)
     when (string-equal (lexical-form-of verb) lexical-form)
     do (return verb)))  
