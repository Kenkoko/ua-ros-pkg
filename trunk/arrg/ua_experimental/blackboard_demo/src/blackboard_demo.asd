(in-package :asdf)

(defsystem "blackboard_demo/blackboard_demo"
    :components ((:file "pkg")
                 (:file "translation" :depends-on ("pkg"))
                 (:file "classes" :depends-on ("translation"))
                 (:file "test" :depends-on ("classes")))
    :depends-on ("roslisp" "gbbopen/gbbopen")) ;; "gazebo_plugins-msg" "std_msgs-msg"))
