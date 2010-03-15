(in-package :blackboard-demo)

(defun test ()
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
    (describe-instance ui))
  (with-ros-node ("test" :spin t)
    (subscribe "update_object" "blackboard_demo/Object" #'add-object)))

(defun add-object (object-msg)
  (print object-msg))