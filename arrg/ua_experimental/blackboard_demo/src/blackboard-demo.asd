(in-package :asdf)

(defsystem "blackboard-demo"
    :components ((:file "pkg")
                 (:file "classes" :depends-on ("pkg"))
                 (:file "test" :depends-on ("classes" "pkg")))
    :depends-on ("roslisp" "gbbopen/gbbopen" "blackboard_demo-msg"))
