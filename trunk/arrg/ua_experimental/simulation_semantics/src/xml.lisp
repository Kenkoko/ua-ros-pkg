(in-package :simulation_semantics)

(defun get-object-path ()
  (let ((proc (sb-ext:run-program "rospack" '("find" "simulator_experiments") 
                                  :wait t :output :stream :search t)))
    (concatenate 'string
                 (read-line (sb-ext:process-output proc))
                 "/objects/")))

;; TODO: probably get rid of this, we are only doing this avoid symbol package problems
;; created by s-xml
;; ... and it doesn't seem to do that anyway
(defparameter *example-xml* 
  (parse-xml-file 
   (concatenate 'string (get-object-path) "blue_box.xml")))
