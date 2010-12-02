
(in-package :asdf)

(defsystem "ultraspeech-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :roslib-msg
)
  :components ((:file "_package")
    (:file "CurrentStim" :depends-on ("_package"))
    (:file "_package_CurrentStim" :depends-on ("_package"))
    (:file "Control" :depends-on ("_package"))
    (:file "_package_Control" :depends-on ("_package"))
    (:file "SaveFile" :depends-on ("_package"))
    (:file "_package_SaveFile" :depends-on ("_package"))
    (:file "AudioStream" :depends-on ("_package"))
    (:file "_package_AudioStream" :depends-on ("_package"))
    ))
