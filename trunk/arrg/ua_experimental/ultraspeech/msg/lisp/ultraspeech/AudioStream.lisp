; Auto-generated. Do not edit!


(in-package ultraspeech-msg)


;//! \htmlinclude AudioStream.msg.html

(defclass <AudioStream> (ros-message)
  ((header
    :reader header-val
    :initarg :header
    :type roslib-msg:<Header>
    :initform (make-instance 'roslib-msg:<Header>))
   (which_channel
    :reader which_channel-val
    :initarg :which_channel
    :type integer
    :initform 0)
   (samples
    :reader samples-val
    :initarg :samples
    :type (vector float)
   :initform (make-array 0 :element-type 'float :initial-element 0.0))
   (num_channels
    :reader num_channels-val
    :initarg :num_channels
    :type integer
    :initform 0)
   (sample_rate
    :reader sample_rate-val
    :initarg :sample_rate
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <AudioStream>) ostream)
  "Serializes a message object of type '<AudioStream>"
  (serialize (slot-value msg 'header) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'which_channel)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'which_channel)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'which_channel)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'which_channel)) ostream)
  (let ((__ros_arr_len (length (slot-value msg 'samples))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele) (let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)))
    (slot-value msg 'samples))
    (write-byte (ldb (byte 8 0) (slot-value msg 'num_channels)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'num_channels)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'num_channels)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'num_channels)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'sample_rate)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'sample_rate)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'sample_rate)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'sample_rate)) ostream)
)
(defmethod deserialize ((msg <AudioStream>) istream)
  "Deserializes a message object of type '<AudioStream>"
  (deserialize (slot-value msg 'header) istream)
  (setf (ldb (byte 8 0) (slot-value msg 'which_channel)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'which_channel)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'which_channel)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'which_channel)) (read-byte istream))
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'samples) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'samples)))
      (dotimes (i __ros_arr_len)
(let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (setf (ldb (byte 8 0) (slot-value msg 'num_channels)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'num_channels)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'num_channels)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'num_channels)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'sample_rate)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'sample_rate)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'sample_rate)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'sample_rate)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<AudioStream>)))
  "Returns string type for a message object of type '<AudioStream>"
  "ultraspeech/AudioStream")
(defmethod md5sum ((type (eql '<AudioStream>)))
  "Returns md5sum for a message object of type '<AudioStream>"
  "e75f13067c4581e6d6ac1ce905ecf22c")
(defmethod message-definition ((type (eql '<AudioStream>)))
  "Returns full string definition for message of type '<AudioStream>"
  (format nil "Header header~%uint32 which_channel~%float32[] samples~%uint32 num_channels~%uint32 sample_rate~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <AudioStream>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     4
     4 (reduce #'+ (slot-value msg 'samples) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
     4
     4
))
(defmethod ros-message-to-list ((msg <AudioStream>))
  "Converts a ROS message object to a list"
  (list '<AudioStream>
    (cons ':header (header-val msg))
    (cons ':which_channel (which_channel-val msg))
    (cons ':samples (samples-val msg))
    (cons ':num_channels (num_channels-val msg))
    (cons ':sample_rate (sample_rate-val msg))
))
