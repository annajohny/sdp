; Auto-generated. Do not edit!


(cl:in-package tug_diagnosis_msgs-msg)


;//! \htmlinclude diagnosis_set.msg.html

(cl:defclass <diagnosis_set> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (type
    :reader type
    :initarg :type
    :type cl:string
    :initform "")
   (diagnoses
    :reader diagnoses
    :initarg :diagnoses
    :type (cl:vector tug_diagnosis_msgs-msg:diagnosis)
   :initform (cl:make-array 0 :element-type 'tug_diagnosis_msgs-msg:diagnosis :initial-element (cl:make-instance 'tug_diagnosis_msgs-msg:diagnosis))))
)

(cl:defclass diagnosis_set (<diagnosis_set>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <diagnosis_set>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'diagnosis_set)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tug_diagnosis_msgs-msg:<diagnosis_set> is deprecated: use tug_diagnosis_msgs-msg:diagnosis_set instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <diagnosis_set>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tug_diagnosis_msgs-msg:header-val is deprecated.  Use tug_diagnosis_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <diagnosis_set>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tug_diagnosis_msgs-msg:type-val is deprecated.  Use tug_diagnosis_msgs-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'diagnoses-val :lambda-list '(m))
(cl:defmethod diagnoses-val ((m <diagnosis_set>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tug_diagnosis_msgs-msg:diagnoses-val is deprecated.  Use tug_diagnosis_msgs-msg:diagnoses instead.")
  (diagnoses m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <diagnosis_set>) ostream)
  "Serializes a message object of type '<diagnosis_set>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'type))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'diagnoses))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'diagnoses))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <diagnosis_set>) istream)
  "Deserializes a message object of type '<diagnosis_set>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'type) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'diagnoses) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'diagnoses)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'tug_diagnosis_msgs-msg:diagnosis))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<diagnosis_set>)))
  "Returns string type for a message object of type '<diagnosis_set>"
  "tug_diagnosis_msgs/diagnosis_set")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'diagnosis_set)))
  "Returns string type for a message object of type 'diagnosis_set"
  "tug_diagnosis_msgs/diagnosis_set")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<diagnosis_set>)))
  "Returns md5sum for a message object of type '<diagnosis_set>"
  "1e8ee6a1d3f192b8969b6d2785e6690b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'diagnosis_set)))
  "Returns md5sum for a message object of type 'diagnosis_set"
  "1e8ee6a1d3f192b8969b6d2785e6690b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<diagnosis_set>)))
  "Returns full string definition for message of type '<diagnosis_set>"
  (cl:format cl:nil "Header header~%string type~%diagnosis[] diagnoses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: tug_diagnosis_msgs/diagnosis~%resource_mode_assignement[] diagnosis~%~%================================================================================~%MSG: tug_diagnosis_msgs/resource_mode_assignement~%string resource~%string mode_msg~%int32 mode~%int32 GENERAL_OK=0~%int32 GENERAL_ERROR=-1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'diagnosis_set)))
  "Returns full string definition for message of type 'diagnosis_set"
  (cl:format cl:nil "Header header~%string type~%diagnosis[] diagnoses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: tug_diagnosis_msgs/diagnosis~%resource_mode_assignement[] diagnosis~%~%================================================================================~%MSG: tug_diagnosis_msgs/resource_mode_assignement~%string resource~%string mode_msg~%int32 mode~%int32 GENERAL_OK=0~%int32 GENERAL_ERROR=-1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <diagnosis_set>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'type))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'diagnoses) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <diagnosis_set>))
  "Converts a ROS message object to a list"
  (cl:list 'diagnosis_set
    (cl:cons ':header (header msg))
    (cl:cons ':type (type msg))
    (cl:cons ':diagnoses (diagnoses msg))
))
