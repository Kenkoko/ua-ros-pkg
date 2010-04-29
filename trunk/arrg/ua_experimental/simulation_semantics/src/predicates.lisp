(in-package :simulation_semantics)

;; TODO: Finish predicate system.

(defmethod annotate-with-predicates ((ws world-state))
  (let* ((predicates (loop for obj1-state in (objects-of ws)
                        collect (compute-my-predicates obj1-state))))
                        ;collect (loop for obj2 in objects 
                        ;           unless (eq obj1 obj2) ;; Don't compute binary predicates with yourself
                        ;           do (compute-binary-predictes obj1 obj2 ws)))))
    (print predicates)))

#+ignore(defun compute-predicates (objects)
  "Generate all pairs of objects x and y, and compute 
   the predicates p(x,y) for each of them"
  (let* ((prm (make-permutator objects objects)))
    (loop for x = (funcall prm)
       until (null x)
       unless (eq (first x) (second x))
       do (print x))))
         
;; HEY! DELETE ALL YOUR FASLS AND SEE IF THAT FIXES IT

;; This sort of needs access to the simulator too, huh? - goals, etc.
(defmethod compute-my-predicates ((my-state object-state))
  (loop with me = (first (object-of my-state))
     for pred in (self-predicates-of me) ;; Why is there a problem with self-predicates-of?
     collect (list pred (funcall pred my-state))))
   
(defun force-mag (os)
  (sqrt (sum-of-squares (linear-of (force-of os)))))
       
(defun sum-of-squares (numbers)
  (loop for x in numbers summing (expt x 2)))

(defmethod distance ((self physical-object) (other physical-object))
  (distance (position-of (pose-of self)) (position-of (pose-of other))))