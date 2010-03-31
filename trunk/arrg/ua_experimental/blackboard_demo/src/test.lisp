(in-package :blackboard_demo)

;;======================================================
;; ROS Node, Publishers and Subscribers

;; Initial translation of the WorldState message class into a GBBopen class
(defun test ()  
  ;; This makes interactive use easier
  (delete-blackboard-repository)
  
  ;; Start a ROS node that stays alive (spins)
  (with-ros-node ("test" :spin t)
    ;; Subscribe to a topic and post translated messages onto the blackboard
    (subscribe "gazebo_world_state" "gazebo_plugins/WorldState" #'translate-msg)
    (subscribe "speech_text" "std_msgs/String" #'handle-speech)

    ;; Start an infinitely looping control shell
    (agenda-shell:start-control-shell :continue-past-quiescence t))
  ;; After the ros node exits, take down the control shell
  (agenda-shell:exit-all-control-shell-threads))

;; Message Callbacks
(defun handle-speech (msg)
  (make-instance 'utterance :sentence (std_msgs-msg:data-val msg)))


;;====================================================
;; Knowledge Sources

(agenda-shell:define-ks utterance-ks
    :trigger-events ((create-instance-event utterance))
    :execution-function 'utterance-response)

(defun utterance-response (ksa)
  (let* ((trigger-utterance (agenda-shell:sole-trigger-instance-of ksa))
         (tokens (simple-split (slot-value trigger-utterance 'sentence) #\ ))
         (verb (parse-verb tokens))
         (goal (parse-goal tokens)))
    (make-instance 'command 
                   :verb-int (make-instance 'interpretation 
                                            :phrase (first verb)
                                            :meaning (second verb))
                   :object-int (make-instance 'interpretation
                                              :phrase goal
                                              :meaning nil)
                   :sentence (slot-value trigger-utterance 'sentence))))
    
(defun parse-goal (tokens)
  (subseq tokens (+ 1 (position "the" tokens :test 'string-equal))))
         
(defun parse-verb (tokens)
  (let* ((verb (subseq tokens 0 2)))
    (cond ((equal '("go" "to") verb) (list verb 'go-to))
          ((equal '("look" "at") verb) (list verb 'look-at))
          (t nil))))
    
;;====================================================

(agenda-shell:define-ks binder-ks
    :trigger-events ((create-instance-event interpretation))
    :activation-predicate 'needs-binding?
    :execution-function 'bind-interpretation)

(defun needs-binding? (ks trigger-event)
  (declare (ignore ks))
  (not (slot-value (instance-of trigger-event) 'meaning)))

(defun bind-interpretation (ksa)
  (print "BINDING")
  (let* ((interp (agenda-shell:sole-trigger-instance-of ksa))
         (phrase (slot-value interp 'phrase))
         (candidates nil))
    (loop for word in phrase
       for matches = (find-matching-objects word)
       when matches do (loop for match in matches do (pushnew match candidates)))
    (if candidates
        (setf (slot-value interp 'meaning) candidates)
        (setf (slot-value interp 'meaning) 'unknown))))

(defun find-matching-objects (word)
  (union 
   (loop for ui in (find-instances-of-class 'object)
     when (string-equal (slot-value ui 'shape) word)
     collect ui)
   (loop for ui in (find-instances-of-class 'object)
      when (string-equal (slot-value ui 'color) word)
      collect ui)))
    
;;====================================================

(agenda-shell:define-ks command-ks
    :trigger-events ((create-instance-event command))
    :execution-function 'do-command)

(defun do-command (ksa)
  (describe-instance (agenda-shell:sole-trigger-instance-of ksa)))

;;====================================================

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
       do (let* ((old-obj (loop for ui in (find-instances-of-class 'Object)
                             when (equal obj-name (slot-value ui 'gazebo-name))
                             return ui)))
            (if old-obj
                (link-setf (location-of old-obj)
                           (make-instance 'position-3d
                                          :x (first obj-xyz)
                                          :y (second obj-xyz)
                                          :z (third obj-xyz)))
                (make-instance 'object 
                               :gazebo-name obj-name 
                               :shape (second obj-desc)
                               :name (third obj-desc) 
                               :color (fourth obj-desc)
                               :location (make-instance 'position-3d
                                                        :x (first obj-xyz)
                                                        :y (second obj-xyz)
                                                        :z (third obj-xyz))))))
    (delete-instance trigger-state)))

;;===================================================
;; Trivial KS Example

(agenda-shell:define-ks initial
     :trigger-events ((agenda-shell:start-control-shell-event)) 
     :execution-function 'initial-ks-function)

(defun initial-ks-function (ksa)
  (declare (ignore ksa))
  (print "STARTED CONTROL SHELL"))


;;====================================================
;; Utility Functions

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