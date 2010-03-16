(in-package :blackboard_demo)

;; Initial translation of the WorldState message class into a GBBopen class
(defun test ()
  (with-ros-node ("test" :spin t)
    (subscribe "gazebo_world_state" "gazebo_plugins/WorldState" #'update-world-state)))

(defun get-simple-name (sym)
  (read-from-string (symbol-name sym)))

(defun collect-slots (class-name)
  (loop for slot-def in (gbbopen-tools:class-direct-slots (find-class class-name))
     collect (read-from-string (symbol-name (gbbopen-tools:slot-definition-name slot-def)))))

(defmacro translate-unit-class (class-name)
  `(define-unit-class ,(intern (symbol-name class-name)) ()
        ,(collect-slots class-name)))

;; Super hacky, need to probably store the symbols that name the generated
;; classes in a hash table or something, appending "blackboard_demo" is not very general
(defun translate-msg (msg)
  (apply #'make-instance 
         (read-from-string (concatenate 'string 
                                        "blackboard_demo::" 
                                        (symbol-name (class-name (class-of msg)))))
         (loop for slot-def in (gbbopen-tools:class-direct-slots (class-of msg))
            for slot-name = (gbbopen-tools:slot-definition-name slot-def)
            append (list (intern (symbol-name slot-name) 'keyword)
                         (slot-value msg slot-name)))))

(defun update-world-state (state-msg)
  (let* ((ui (translate-msg state-msg)))
    (describe-instance ui)))

(defun object-test ()
  (let* ((ui (make-instance 'object 
                            :name "red_cube_1"
                            :color 'red
                            :shape 'cube
                            :location (make-instance 'position-3d
                                                     :x 0.5
                                                     :y 1.5
                                                     :z 2.0))))
    (add-instance-to-space-instance ui *object-space*)
    (describe-blackboard-repository)
    (describe-instance ui)))
