(in-package :simsem)

;;======================================================================
;; Interventions

#+ignore(defun new-global-intervention ()
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

