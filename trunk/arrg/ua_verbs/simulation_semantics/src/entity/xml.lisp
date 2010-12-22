(in-package :simulation_semantics)

(defun get-object-path ()
  (let ((proc (sb-ext:run-program "rospack" '("find" "simulation_semantics") 
                                  :wait t :output :stream :search t)))
    (concatenate 'string
                 (read-line (sb-ext:process-output proc))
                 "/objects/")))

;; Seems like we do actually need this?
(parse-xml-file 
 (concatenate 'string (get-object-path) "blue_box.xml"))

;; But do we really need this?
(parse-xml-file 
 (concatenate 'string (get-object-path) "blue_sphere.xml"))

(defparameter *point-xml* 
  (print-xml-string (parse-xml-file
                     (concatenate 'string (get-object-path) "point.xml"))
                    :pretty t))
