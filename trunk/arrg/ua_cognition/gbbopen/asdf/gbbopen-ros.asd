;; GBBopen has a customized .asd file, depending on this system should 
;; ensure that other packages can use GBBopen transparently.

(require :gbbopen)

(defun get-gbbopen-initiate ()
  (let ((proc (sb-ext:run-program "rospack" '("find" "gbbopen") 
                                     :wait t :output :stream :search t)))
    (concatenate 'string
                 (read-line (sb-ext:process-output proc))
                 "/gbbopen/initiate.lisp")))
(load (get-gbbopen-initiate))
(cl-user::gbbopen-user)

(in-package :asdf)

(defsystem :gbbopen-ros
    :depends-on (:gbbopen :gbbopen-user :agenda-shell-user))