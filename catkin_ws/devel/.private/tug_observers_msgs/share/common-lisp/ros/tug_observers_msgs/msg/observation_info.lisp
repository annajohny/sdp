; Auto-generated. Do not edit!


(cl:in-package tug_observers_msgs-msg)


;//! \htmlinclude observation_info.msg.html

(cl:defclass <observation_info> (roslisp-msg-protocol:ros-message)
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
   (resource
    :reader resource
    :initarg :resource
    :type cl:string
    :initform "")
   (observation
    :reader observation
    :initarg :observation
    :type (cl:vector tug_observers_msgs-msg:observation)
   :initform (cl:make-array 0 :element-type 'tug_observers_msgs-msg:observation :initial-element (cl:make-instance 'tug_observers_msgs-msg:observation))))
)

(cl:defclass observation_info (<observation_info>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <observation_info>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'observation_info)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tug_observers_msgs-msg:<observation_info> is deprecated: use tug_observers_msgs-msg:observation_info instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <observation_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tug_observers_msgs-msg:header-val is deprecated.  Use tug_observers_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <observation_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tug_observers_msgs-msg:type-val is deprecated.  Use tug_observers_msgs-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'resource-val :lambda-list '(m))
(cl:defmethod resource-val ((m <observation_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tug_observers_msgs-msg:resource-val is deprecated.  Use tug_observers_msgs-msg:resource instead.")
  (resource m))

(cl:ensure-generic-function 'observation-val :lambda-list '(m))
(cl:defmethod observation-val ((m <observation_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tug_observers_msgs-msg:observation-val is deprecated.  Use tug_observers_msgs-msg:observation instead.")
  (observation m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <observation_info>) ostream)
  "Serializes a message object of type '<observation_info>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'type))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'resource))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'resource))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'observation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'observation))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <observation_info>) istream)
  "Deserializes a message object of type '<observation_info>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'type) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'resource) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'resource) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'observation) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'observation)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'tug_observers_msgs-msg:observation))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<observation_info>)))
  "Returns string type for a message object of type '<observation_info>"
  "tug_observers_msgs/observation_info")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'observation_info)))
  "Returns string type for a message object of type 'observation_info"
  "tug_observers_msgs/observation_info")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<observation_info>)))
  "Returns md5sum for a message object of type '<observation_info>"
  "920b39a3d493095fc494ca757b21762f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'observation_info)))
  "Returns md5sum for a message object of type 'observation_info"
  "920b39a3d493095fc494ca757b21762f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<observation_info>)))
  "Returns full string definition for message of type '<observation_info>"
  (cl:format cl:nil "Header header~%string type~%string resource~%observation[] observation~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: tug_observers_msgs/observation~%string observation_msg~%string verbose_observation_msg~%int32 observation~%int32 GENERAL_OK=0~%int32 GENERAL_ERROR=-1~%int32 NO_STATE_FITS=-2~%int32 NOT_AVAILABLE=-3~%int32 TIMEOUT=-4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'observation_info)))
  "Returns full string definition for message of type 'observation_info"
  (cl:format cl:nil "Header header~%string type~%string resource~%observation[] observation~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: tug_observers_msgs/observation~%string observation_msg~%string verbose_observation_msg~%int32 observation~%int32 GENERAL_OK=0~%int32 GENERAL_ERROR=-1~%int32 NO_STATE_FITS=-2~%int32 NOT_AVAILABLE=-3~%int32 TIMEOUT=-4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <observation_info>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'type))
     4 (cl:length (cl:slot-value msg 'resource))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'observation) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <observation_info>))
  "Converts a ROS message object to a list"
  (cl:list 'observation_info
    (cl:cons ':header (header msg))
    (cl:cons ':type (type msg))
    (cl:cons ':resource (resource msg))
    (cl:cons ':observation (observation msg))
))
