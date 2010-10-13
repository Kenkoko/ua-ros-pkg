(in-package :simulation_semantics)

(defun save-all-instances (&rest classes)
  (with-open-file (stream "test.bb" :direction :output :if-exists :supersede)
    (with-saving/sending-block (stream)
      (setf *save/send-references-only* nil)
      (loop for class in classes 
         do (loop for inst in (find-instances-of-class class)
               do (print-object-for-saving/sending inst stream))))))

;; TODO: Probably move to utils
(defun fibn (name)
  "shortcut for (find-instance-by-name name)"
  (find-instance-by-name name)) 

(defun delete-all-instances (&rest classes)
  (loop for class in classes 
     for instances = (find-instances-of-class class)
     do (loop for instance in instances 
           do (delete-instance instance))))

(defun delete-simulation-data ()
  (setf *current-state* nil)
  (delete-all-instances 'force 'object-state 'pose 'quaternion 'velocity  'xyz)
  (loop for sim in (find-instances-of-class 'simulator)
     for si = (find-space-instance-by-path (list (instance-name-of sim)))
     when si do (delete-space-instance si)))

(defun save-library ()
  (delete-simulation-data)
  (save-blackboard-repository "test.bb"))

(defun restore ()
  ;(delete-blackboard-repository)
  ;(make-object-space)
  ;(make-simulator-space)
  (load-blackboard-repository "test.bb"))
