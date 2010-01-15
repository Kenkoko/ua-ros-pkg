
(in-package :asdf)

(defsystem "wubble_environments-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils)
  :components ((:file "_package")
    (:file "IcarusWorldState" :depends-on ("_package"))
    (:file "_package_IcarusWorldState" :depends-on ("_package"))
    ))
