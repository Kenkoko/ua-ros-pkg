; Auto-generated. Do not edit!


(in-package ultraspeech-msg)


;//! \htmlinclude SaveFile.msg.html

(defclass <SaveFile> (ros-message)
  ((header
    :reader header-val
    :initarg :header
    :type roslib-msg:<Header>
    :initform (make-instance 'roslib-msg:<Header>))
   (filepath
    :reader filepath-val
    :initarg :filepath
    :type string
    :initform ""))
)
(defmethod serialize ((msg <SaveFile>) ostream)
  "Serializes a message object of type '<SaveFile>"
  (serialize (slot-value msg 'header) ostream)
  (let ((__ros_str_len (length (slot-value msg 'filepath))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'filepath))
)
(defmethod deserialize ((msg <SaveFile>) istream)
  "Deserializes a message object of type '<SaveFile>"
  (deserialize (slot-value msg 'header) istream)
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'filepath) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'filepath) __ros_str_idx) (code-char (read-byte istream)))))
  msg
)
(defmethod ros-datatype ((msg (eql '<SaveFile>)))
  "Returns string type for a message object of type '<SaveFile>"
  "ultraspeech/SaveFile")
(defmethod md5sum ((type (eql '<SaveFile>)))
  "Returns md5sum for a message object of type '<SaveFile>"
  "02eadefcd88df0f76162b21327c4bd37")
(defmethod message-definition ((type (eql '<SaveFile>)))
  "Returns full string definition for message of type '<SaveFile>"
  (format nil "Header header~%string filepath~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <SaveFile>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     4 (length (slot-value msg 'filepath))
))
(defmethod ros-message-to-list ((msg <SaveFile>))
  "Converts a ROS message object to a list"
  (list '<SaveFile>
    (cons ':header (header-val msg))
    (cons ':filepath (filepath-val msg))
))
