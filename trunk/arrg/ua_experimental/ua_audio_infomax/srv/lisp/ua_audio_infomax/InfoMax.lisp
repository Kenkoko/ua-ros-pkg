; Auto-generated. Do not edit!


(in-package ua_audio_infomax-srv)


;//! \htmlinclude InfoMax-request.msg.html

(defclass <InfoMax-request> (ros-message)
  ((objectNames
    :reader objectNames-val
    :initarg :objectNames
    :type (vector string)
   :initform (make-array 0 :element-type 'string :initial-element ""))
   (actionNames
    :reader actionNames-val
    :initarg :actionNames
    :type (vector string)
   :initform (make-array 0 :element-type 'string :initial-element ""))
   (actionID
    :reader actionID-val
    :initarg :actionID
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <InfoMax-request>) ostream)
  "Serializes a message object of type '<InfoMax-request>"
  (let ((__ros_arr_len (length (slot-value msg 'objectNames))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele) (let ((__ros_str_len (length ele)))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) ele))
    (slot-value msg 'objectNames))
  (let ((__ros_arr_len (length (slot-value msg 'actionNames))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele) (let ((__ros_str_len (length ele)))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) ele))
    (slot-value msg 'actionNames))
    (write-byte (ldb (byte 8 0) (slot-value msg 'actionID)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'actionID)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'actionID)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'actionID)) ostream)
)
(defmethod deserialize ((msg <InfoMax-request>) istream)
  "Deserializes a message object of type '<InfoMax-request>"
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'objectNames) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'objectNames)))
      (dotimes (i __ros_arr_len)
(let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (aref vals i) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (aref vals i) __ros_str_idx) (code-char (read-byte istream))))))))
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'actionNames) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'actionNames)))
      (dotimes (i __ros_arr_len)
(let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (aref vals i) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (aref vals i) __ros_str_idx) (code-char (read-byte istream))))))))
  (setf (ldb (byte 8 0) (slot-value msg 'actionID)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'actionID)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'actionID)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'actionID)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<InfoMax-request>)))
  "Returns string type for a service object of type '<InfoMax-request>"
  "ua_audio_infomax/InfoMaxRequest")
(defmethod md5sum ((type (eql '<InfoMax-request>)))
  "Returns md5sum for a message object of type '<InfoMax-request>"
  "796ea21d4f970e10e46e5cd3084ae919")
(defmethod message-definition ((type (eql '<InfoMax-request>)))
  "Returns full string definition for message of type '<InfoMax-request>"
  (format nil "# request~%~%#std_msgs/String[] objectNames		# names of all objects~%string[] objectNames				# names of all objects~%#int32 objectID						# index of chosen object~%#std_msgs/String[] actionNames		~%string[] actionNames				# names of all actions: move left, move right, pick up, drop, push, squeeze, reset~%int32 actionID 						# index of action to take~%~%~%"))
(defmethod serialization-length ((msg <InfoMax-request>))
  (+ 0
     4 (reduce #'+ (slot-value msg 'objectNames) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4 (length ele))))
     4 (reduce #'+ (slot-value msg 'actionNames) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4 (length ele))))
     4
))
(defmethod ros-message-to-list ((msg <InfoMax-request>))
  "Converts a ROS message object to a list"
  (list '<InfoMax-request>
    (cons ':objectNames (objectNames-val msg))
    (cons ':actionNames (actionNames-val msg))
    (cons ':actionID (actionID-val msg))
))
;//! \htmlinclude InfoMax-response.msg.html

(defclass <InfoMax-response> (ros-message)
  ((beliefs
    :reader beliefs-val
    :initarg :beliefs
    :type (vector float)
   :initform (make-array 0 :element-type 'float :initial-element 0.0))
   (location
    :reader location-val
    :initarg :location
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <InfoMax-response>) ostream)
  "Serializes a message object of type '<InfoMax-response>"
  (let ((__ros_arr_len (length (slot-value msg 'beliefs))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele) (let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream)))
    (slot-value msg 'beliefs))
    (write-byte (ldb (byte 8 0) (slot-value msg 'location)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'location)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'location)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'location)) ostream)
)
(defmethod deserialize ((msg <InfoMax-response>) istream)
  "Deserializes a message object of type '<InfoMax-response>"
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'beliefs) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'beliefs)))
      (dotimes (i __ros_arr_len)
(let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (setf (ldb (byte 8 0) (slot-value msg 'location)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'location)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'location)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'location)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<InfoMax-response>)))
  "Returns string type for a service object of type '<InfoMax-response>"
  "ua_audio_infomax/InfoMaxResponse")
(defmethod md5sum ((type (eql '<InfoMax-response>)))
  "Returns md5sum for a message object of type '<InfoMax-response>"
  "796ea21d4f970e10e46e5cd3084ae919")
(defmethod message-definition ((type (eql '<InfoMax-response>)))
  "Returns full string definition for message of type '<InfoMax-response>"
  (format nil "# response~%~%float64[] beliefs	# PDF over classes and objects, conditioned on action~%int32 location		# current location of robot (integer index, should be the same as the object index)~%~%~%"))
(defmethod serialization-length ((msg <InfoMax-response>))
  (+ 0
     4 (reduce #'+ (slot-value msg 'beliefs) :key #'(lambda (ele) (declare (ignorable ele)) (+ 8)))
     4
))
(defmethod ros-message-to-list ((msg <InfoMax-response>))
  "Converts a ROS message object to a list"
  (list '<InfoMax-response>
    (cons ':beliefs (beliefs-val msg))
    (cons ':location (location-val msg))
))
(defmethod service-request-type ((msg (eql 'InfoMax)))
  '<InfoMax-request>)
(defmethod service-response-type ((msg (eql 'InfoMax)))
  '<InfoMax-response>)
(defmethod ros-datatype ((msg (eql 'InfoMax)))
  "Returns string type for a service object of type '<InfoMax>"
  "ua_audio_infomax/InfoMax")
