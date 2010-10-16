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

(defun make-verb-description (verb)
  (make-message "verb_learning/VerbDescription"
                :verb (lexical-form-of verb)
                :arguments (ros-list (argument-strings verb))))

(defun make-verb-instance (verb scenario)
  (make-message "verb_learning/VerbInstance"
                :verb (lexical-form-of verb)
                :arguments (ros-list (argument-strings verb))
                :bindings (ros-list (loop for object in (argument-list-of scenario)
                                       collect (gazebo-name-of object)))))
                
(defun present-scenario (simulator arguments)
  "Arguments is a list that has the object names in the appropriate argument order"
  (push (make-instance 'scenario :argument-list arguments :simulator simulator)
        (scenarios-of *current-verb*)))
  

        
(defun find-verb (word)
  (loop for verb in (find-instances-of-class 'verb)
     if (string-equal (lexical-form-of verb) word)
     do (return verb)))
       
(defun blast ()
  (delete-all-instances 'verb 'simulator 'scenario))
         
#+ignore(defun fsm-test (&key (scenario *current-scenario*) (verb *current-verb*))
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

(defun get-episodes (verb)
  (loop for scenario in (scenarios-of verb)
     append (loop for trace in (traces-of (simulator-of scenario))
               for episode = (intervals-to-episode 
                              (pmts-to-intervals 
                               (convert-trace-to-pmts trace (name-map-of scenario verb))))
               collect episode)))

(defun trace-to-episode (trace name-map)
  (intervals-to-episode 
   (pmts-to-intervals 
    (convert-trace-to-pmts trace name-map))))

;;======================================================================
;; Scenario Configuration

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
