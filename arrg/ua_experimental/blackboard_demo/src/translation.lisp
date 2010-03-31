(in-package :blackboard_demo)

(defparameter *msg->ut* (make-hash-table :test 'eq))

;; TODO: Also, make it so that this translates the slot values 
;; that are messages into gbbopen objects - if possible
(defun translate-msg (msg)
  ;(setf *msg* msg)
  (apply #'make-instance 
         (message-class-to-unit-class (class-of msg))
         ;(find-symbol (symbol-name (class-name (class-of msg))) 
         ;             (find-package 'blackboard_demo))
         (loop for slot-def in (gbbopen-tools:class-direct-slots (class-of msg))
            for slot-name = (gbbopen-tools:slot-definition-name slot-def)
            append (list (intern (symbol-name slot-name) 'keyword)
                         (slot-value msg slot-name)))))

(defun message-class-to-unit-class (msg-class)
  (let* ((unit-class (gethash msg-class *msg->ut*)))
    (cond (unit-class unit-class)
          (t (setf (gethash msg-class *msg->ut*) 
                   (translate-unit-class msg-class))
             (gethash msg-class *msg->ut*)))))

(defun message-type-to-unit-class (pkg msg)
  (let* ((asdf-str (concatenate 'string pkg "-msg")))
    ;(print asdf-str)
    (asdf:operate 'asdf:load-op asdf-str)
    (let* ((pkg-str (string-upcase asdf-str))
           (pkg-msg (find-package (intern pkg-str :keyword)))
           (msg-sym (find-symbol (string-upcase (concatenate 'string "<" msg ">")) pkg-msg))
           (msg-class (find-class msg-sym)))
      (message-class-to-unit-class msg-class))))

(defun translate-unit-class (msg-class)
  (let* ((uc-name (intern (symbol-name (class-name msg-class)))))
    (eval `(define-unit-class ,uc-name ()
             ,(collect-slots msg-class)))))

(defun collect-slots (some-class)
  (loop for slot-def in (gbbopen-tools:class-direct-slots some-class)
     ;do (print (gbbopen-tools:slot-definition-name slot-def))
     collect (read-from-string (symbol-name (gbbopen-tools:slot-definition-name slot-def)))))
