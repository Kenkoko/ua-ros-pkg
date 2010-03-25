(in-package :blackboard_demo)

(defparameter *msg* nil)

(message-type-to-unit-class "gazebo_plugins" "WorldState")

(define-unit-class object () 
  (gazebo-name 
   name 
   shape 
   color
   (location 
    :link (position-3d objects-here)))
  (:initial-space-instances (objects)))

(define-unit-class position-3d ()
  (x y z
     (objects-here 
      :link (object location)))
  (:dimensional-values
      (x :point x)
      (y :point y)
      (z :point z)))

;; Initial translation of the WorldState message class into a GBBopen class
(defun test ()  
  (make-space-instance '(objects))
  (with-ros-node ("test" :spin t)
    ;; Subscribe to a topic and post translated messages onto the blackboard
    (subscribe "gazebo_world_state" "gazebo_plugins/WorldState" #'translate-msg)
    ;; Start and infinitely looping control shell
    (agenda-shell:start-control-shell :continue-past-quiescence t))
  (agenda-shell:exit-control-shell ':solution-found t))

(defun update-world-state (state-msg)
  (let* ((ui (translate-msg state-msg)))
    (describe-instance ui)))

;; Knowledge source test.
(agenda-shell:define-ks world-state-ks
    :trigger-events ((create-instance-event <WorldState>))
    :execution-function 'world-state-response)

(defun world-state-response (ksa)
  (print "GOT WORLD STATE")
  (let* ((trigger-state (agenda-shell:sole-trigger-instance-of ksa)))
    ;(describe-instance trigger-state)))
    (loop for i below (array-dimension (name-of trigger-state) 0)
       for obj-name = (aref (name-of trigger-state) i)
       for obj-desc = (simple-split obj-name #\_)
       for obj-pose = (aref (pose-of trigger-state) i)
       for obj-xyz = (extract-xyz obj-pose)
       when (search "obj" obj-name)
       do (print obj-xyz)
        (describe-instance 
         (make-instance 'object 
                        :gazebo-name obj-name 
                        :shape (second obj-desc)
                        :name (third obj-desc) 
                        :color (fourth obj-desc)
                        :location (make-instance 'position-3d
                                                 :x (first obj-xyz)
                                                 :y (second obj-xyz)
                                                 :z (third obj-xyz)))))
    (delete-instance trigger-state)))

(agenda-shell:define-ks initial
     :trigger-events ((agenda-shell:start-control-shell-event)) 
     :execution-function #'initial-ks-function)
 
(defun initial-ks-function (ksa)
  (declare (ignore ksa))
  (print "STARTED CONTROL SHELL"))


(defun simple-split (string divider)
    "Returns a list of substrings of string divided by ONE divider each.
     Note: Two consecutive spaces will be seen as if there were an empty string between them."
    (loop for i = 0 then (1+ j)
          as j = (position divider string :start i)
          collect (subseq string i j)
          while j))

(defun extract-xyz (pose-msg)
  (let* ((xyz-list (rest (roslisp-msg-protocol:ros-message-to-list 
                          (geometry_msgs-msg:position-val pose-msg)))))
    (list (cdr (assoc :x xyz-list))
          (cdr (assoc :y xyz-list))
          (cdr (assoc :z xyz-list)))))
          