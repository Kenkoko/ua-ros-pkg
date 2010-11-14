; Auto-generated. Do not edit!


(in-package ultraspeech-msg)


;//! \htmlinclude CurrentStim.msg.html

(defclass <CurrentStim> (ros-message)
  ((header
    :reader header-val
    :initarg :header
    :type roslib-msg:<Header>
    :initform (make-instance 'roslib-msg:<Header>))
   (stimulus
    :reader stimulus-val
    :initarg :stimulus
    :type string
    :initform "")
   (rep
    :reader rep-val
    :initarg :rep
    :type fixnum
    :initform 0)
   (batch
    :reader batch-val
    :initarg :batch
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <CurrentStim>) ostream)
  "Serializes a message object of type '<CurrentStim>"
  (serialize (slot-value msg 'header) ostream)
  (let ((__ros_str_len (length (slot-value msg 'stimulus))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'stimulus))
    (write-byte (ldb (byte 8 0) (slot-value msg 'rep)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'batch)) ostream)
)
(defmethod deserialize ((msg <CurrentStim>) istream)
  "Deserializes a message object of type '<CurrentStim>"
  (deserialize (slot-value msg 'header) istream)
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'stimulus) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'stimulus) __ros_str_idx) (code-char (read-byte istream)))))
  (setf (ldb (byte 8 0) (slot-value msg 'rep)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'batch)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<CurrentStim>)))
  "Returns string type for a message object of type '<CurrentStim>"
  "ultraspeech/CurrentStim")
(defmethod md5sum ((type (eql '<CurrentStim>)))
  "Returns md5sum for a message object of type '<CurrentStim>"
  "d1b9396164e302ceb1d840a0e5c4d3b0")
(defmethod message-definition ((type (eql '<CurrentStim>)))
  "Returns full string definition for message of type '<CurrentStim>"
  (format nil "Header header~%string stimulus~%int8 rep~%int8 batch~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <CurrentStim>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     4 (length (slot-value msg 'stimulus))
     1
     1
))
(defmethod ros-message-to-list ((msg <CurrentStim>))
  "Converts a ROS message object to a list"
  (list '<CurrentStim>
    (cons ':header (header-val msg))
    (cons ':stimulus (stimulus-val msg))
    (cons ':rep (rep-val msg))
    (cons ':batch (batch-val msg))
))
