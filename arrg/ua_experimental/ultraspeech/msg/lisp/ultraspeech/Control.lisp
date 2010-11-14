; Auto-generated. Do not edit!


(in-package ultraspeech-msg)


;//! \htmlinclude Control.msg.html

(defclass <Control> (ros-message)
  ((header
    :reader header-val
    :initarg :header
    :type roslib-msg:<Header>
    :initform (make-instance 'roslib-msg:<Header>))
   (run
    :reader run-val
    :initarg :run
    :type fixnum
    :initform 0)
   (directory
    :reader directory-val
    :initarg :directory
    :type string
    :initform ""))
)
(defmethod serialize ((msg <Control>) ostream)
  "Serializes a message object of type '<Control>"
  (serialize (slot-value msg 'header) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'run)) ostream)
  (let ((__ros_str_len (length (slot-value msg 'directory))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'directory))
)
(defmethod deserialize ((msg <Control>) istream)
  "Deserializes a message object of type '<Control>"
  (deserialize (slot-value msg 'header) istream)
  (setf (ldb (byte 8 0) (slot-value msg 'run)) (read-byte istream))
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'directory) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'directory) __ros_str_idx) (code-char (read-byte istream)))))
  msg
)
(defmethod ros-datatype ((msg (eql '<Control>)))
  "Returns string type for a message object of type '<Control>"
  "ultraspeech/Control")
(defmethod md5sum ((type (eql '<Control>)))
  "Returns md5sum for a message object of type '<Control>"
  "122bda80d342152b54964353521a1ca8")
(defmethod message-definition ((type (eql '<Control>)))
  "Returns full string definition for message of type '<Control>"
  (format nil "Header header~%int8 run~%string directory~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <Control>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     1
     4 (length (slot-value msg 'directory))
))
(defmethod ros-message-to-list ((msg <Control>))
  "Converts a ROS message object to a list"
  (list '<Control>
    (cons ':header (header-val msg))
    (cons ':run (run-val msg))
    (cons ':directory (directory-val msg))
))
