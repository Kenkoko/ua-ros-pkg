; Auto-generated. Do not edit!


(in-package wubble_environments-srv)


;//! \htmlinclude IcarusWorldState-request.msg.html

(defclass <IcarusWorldState-request> (ros-message)
  ()
)
(defmethod serialize ((msg <IcarusWorldState-request>) ostream)
  "Serializes a message object of type '<IcarusWorldState-request>"
)
(defmethod deserialize ((msg <IcarusWorldState-request>) istream)
  "Deserializes a message object of type '<IcarusWorldState-request>"
  msg
)
(defmethod ros-datatype ((msg (eql '<IcarusWorldState-request>)))
  "Returns string type for a service object of type '<IcarusWorldState-request>"
  "wubble_environments/IcarusWorldStateRequest")
(defmethod md5sum ((type (eql '<IcarusWorldState-request>)))
  "Returns md5sum for a message object of type '<IcarusWorldState-request>"
  #xaf6d3a99f0fbeb66d3248fa4b3e675fb)
(defmethod message-definition ((type (eql '<IcarusWorldState-request>)))
  "Returns full string definition for message of type '<IcarusWorldState-request>"
  (format nil "~%"))
(defmethod serialization-length ((msg <IcarusWorldState-request>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <IcarusWorldState-request>))
  "Converts a ROS message object to a list"
  (list '<IcarusWorldState-request>
))
;//! \htmlinclude IcarusWorldState-response.msg.html

(defclass <IcarusWorldState-response> (ros-message)
  ((state
    :accessor state-val
    :initarg :state
    :initform ""))
)
(defmethod serialize ((msg <IcarusWorldState-response>) ostream)
  "Serializes a message object of type '<IcarusWorldState-response>"
  (let ((__ros_str_len (length (slot-value msg 'state))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'state))
)
(defmethod deserialize ((msg <IcarusWorldState-response>) istream)
  "Deserializes a message object of type '<IcarusWorldState-response>"
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'state) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'state) __ros_str_idx) (code-char (read-byte istream)))))
  msg
)
(defmethod ros-datatype ((msg (eql '<IcarusWorldState-response>)))
  "Returns string type for a service object of type '<IcarusWorldState-response>"
  "wubble_environments/IcarusWorldStateResponse")
(defmethod md5sum ((type (eql '<IcarusWorldState-response>)))
  "Returns md5sum for a message object of type '<IcarusWorldState-response>"
  #xaf6d3a99f0fbeb66d3248fa4b3e675fb)
(defmethod message-definition ((type (eql '<IcarusWorldState-response>)))
  "Returns full string definition for message of type '<IcarusWorldState-response>"
  (format nil "string state~%~%~%"))
(defmethod serialization-length ((msg <IcarusWorldState-response>))
  (+ 0
     4 (length (slot-value msg 'state))
))
(defmethod ros-message-to-list ((msg <IcarusWorldState-response>))
  "Converts a ROS message object to a list"
  (list '<IcarusWorldState-response>
    (cons ':state (ros-message-to-list (state-val msg)))
))
(defmethod service-request-type ((msg (eql 'IcarusWorldState)))
  '<IcarusWorldState-request>)
(defmethod service-response-type ((msg (eql 'IcarusWorldState)))
  '<IcarusWorldState-response>)
(defmethod ros-datatype ((msg (eql 'IcarusWorldState)))
  "Returns string type for a service object of type '<IcarusWorldState>"
  "wubble_environments/IcarusWorldState")
