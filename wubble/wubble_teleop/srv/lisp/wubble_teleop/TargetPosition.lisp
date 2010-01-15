; Auto-generated. Do not edit!


(in-package wubble_teleop-srv)


;//! \htmlinclude TargetPosition-request.msg.html

(defclass <TargetPosition-request> (ros-message)
  ((x
    :accessor x-val
    :initarg :x
    :initform 0.0)
   (y
    :accessor y-val
    :initarg :y
    :initform 0.0)
   (z
    :accessor z-val
    :initarg :z
    :initform 0.0))
)
(defmethod serialize ((msg <TargetPosition-request>) ostream)
  "Serializes a message object of type '<TargetPosition-request>"
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'x))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'y))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'z))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
)
(defmethod deserialize ((msg <TargetPosition-request>) istream)
  "Deserializes a message object of type '<TargetPosition-request>"
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'x) (roslisp-utils:decode-double-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'y) (roslisp-utils:decode-double-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'z) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<TargetPosition-request>)))
  "Returns string type for a service object of type '<TargetPosition-request>"
  "wubble_teleop/TargetPositionRequest")
(defmethod md5sum ((type (eql '<TargetPosition-request>)))
  "Returns md5sum for a message object of type '<TargetPosition-request>"
  #x14fb54e9e518f55d418823395ca25d0b)
(defmethod message-definition ((type (eql '<TargetPosition-request>)))
  "Returns full string definition for message of type '<TargetPosition-request>"
  (format nil "float64 x~%float64 y~%float64 z~%~%"))
(defmethod serialization-length ((msg <TargetPosition-request>))
  (+ 0
     8
     8
     8
))
(defmethod ros-message-to-list ((msg <TargetPosition-request>))
  "Converts a ROS message object to a list"
  (list '<TargetPosition-request>
    (cons ':x (ros-message-to-list (x-val msg)))
    (cons ':y (ros-message-to-list (y-val msg)))
    (cons ':z (ros-message-to-list (z-val msg)))
))
;//! \htmlinclude TargetPosition-response.msg.html

(defclass <TargetPosition-response> (ros-message)
  ((success
    :accessor success-val
    :initarg :success
    :initform nil))
)
(defmethod serialize ((msg <TargetPosition-response>) ostream)
  "Serializes a message object of type '<TargetPosition-response>"
    (write-byte (ldb (byte 8 0) (if (slot-value msg 'success) 1 0)) ostream)
)
(defmethod deserialize ((msg <TargetPosition-response>) istream)
  "Deserializes a message object of type '<TargetPosition-response>"
  (setf (slot-value msg 'success) (not (zerop (read-byte istream))))
  msg
)
(defmethod ros-datatype ((msg (eql '<TargetPosition-response>)))
  "Returns string type for a service object of type '<TargetPosition-response>"
  "wubble_teleop/TargetPositionResponse")
(defmethod md5sum ((type (eql '<TargetPosition-response>)))
  "Returns md5sum for a message object of type '<TargetPosition-response>"
  #x14fb54e9e518f55d418823395ca25d0b)
(defmethod message-definition ((type (eql '<TargetPosition-response>)))
  "Returns full string definition for message of type '<TargetPosition-response>"
  (format nil "bool success~%~%~%"))
(defmethod serialization-length ((msg <TargetPosition-response>))
  (+ 0
     1
))
(defmethod ros-message-to-list ((msg <TargetPosition-response>))
  "Converts a ROS message object to a list"
  (list '<TargetPosition-response>
    (cons ':success (ros-message-to-list (success-val msg)))
))
(defmethod service-request-type ((msg (eql 'TargetPosition)))
  '<TargetPosition-request>)
(defmethod service-response-type ((msg (eql 'TargetPosition)))
  '<TargetPosition-response>)
(defmethod ros-datatype ((msg (eql 'TargetPosition)))
  "Returns string type for a service object of type '<TargetPosition>"
  "wubble_teleop/TargetPosition")
