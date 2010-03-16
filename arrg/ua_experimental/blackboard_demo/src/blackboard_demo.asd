(in-package :asdf)

(defsystem "blackboard_demo"
    :components ((:file "pkg")
                 (:file "classes" :depends-on ("pkg"))
                 (:file "test" :depends-on ("classes" "pkg")))
    :depends-on ("roslisp" "gbbopen/gbbopen" "gazebo_plugins-msg"))
