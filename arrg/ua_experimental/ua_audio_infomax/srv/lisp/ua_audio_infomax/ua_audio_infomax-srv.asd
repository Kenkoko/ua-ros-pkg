
(in-package :asdf)

(defsystem "ua_audio_infomax-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils)
  :components ((:file "_package")
    (:file "InfoMax" :depends-on ("_package"))
    (:file "_package_InfoMax" :depends-on ("_package"))
    ))
