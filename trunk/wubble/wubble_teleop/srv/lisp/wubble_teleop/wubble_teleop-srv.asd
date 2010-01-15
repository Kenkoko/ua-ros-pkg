
(in-package :asdf)

(defsystem "wubble_teleop-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils)
  :components ((:file "_package")
    (:file "TargetPosition" :depends-on ("_package"))
    (:file "_package_TargetPosition" :depends-on ("_package"))
    ))
