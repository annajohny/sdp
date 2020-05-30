; Auto-generated. Do not edit!


(cl:in-package tug_observers_msgs-msg)


;//! \htmlinclude observer_info.msg.html

(cl:defclass <observer_info> (roslisp-msg-protocol:ros-message)
  ((observation_infos
    :reader observation_infos
    :initarg :observation_infos
    :type (cl:vector tug_observers_msgs-msg:observation_info)
   :initform (cl:make-array 0 :element-type 'tug_observers_msgs-msg:observation_info :initial-element (cl:make-instance 'tug_observers_msgs-msg:observation_info))))
)

(cl:defclass observer_info (<observer_info>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <observer_info>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'observer_info)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tug_observers_msgs-msg:<observer_info> is deprecated: use tug_observers_msgs-msg:observer_info instead.")))

(cl:ensure-generic-function 'observation_infos-val :lambda-list '(m))
(cl:defmethod observation_infos-val ((m <observer_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tug_observers_msgs-msg:observation_infos-val is deprecated.  Use tug_observers_msgs-msg:observation_infos instead.")
  (observation_infos m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <observer_info>) ostream)
  "Serializes a message object of type '<observer_info>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'observation_infos))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'observation_infos))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <observer_info>) istream)
  "Deserializes a message object of type '<observer_info>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'observation_infos) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'observation_infos)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'tug_observers_msgs-msg:observation_info))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<observer_info>)))
  "Returns string type for a message object of type '<observer_info>"
  "tug_observers_msgs/observer_info")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'observer_info)))
  "Returns string type for a message object of type 'observer_info"
  "tug_observers_msgs/observer_info")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<observer_info>)))
  "Returns md5sum for a message object of type '<observer_info>"
  "38c26f4d3dc2b7fc8f36eef35fb2083c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'observer_info)))
  "Returns md5sum for a message object of type 'observer_info"
  "38c26f4d3dc2b7fc8f36eef35fb2083c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<observer_info>)))
  "Returns full string definition for message of type '<observer_info>"
  (cl:format cl:nil "observation_info[] observation_infos~%~%================================================================================~%MSG: tug_observers_msgs/observation_info~%Header header~%string type~%string resource~%observation[] observation~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: tug_observers_msgs/observation~%string observation_msg~%string verbose_observation_msg~%int32 observation~%int32 GENERAL_OK=0~%int32 GENERAL_ERROR=-1~%int32 NO_STATE_FITS=-2~%int32 NOT_AVAILABLE=-3~%int32 TIMEOUT=-4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'observer_info)))
  "Returns full string definition for message of type 'observer_info"
  (cl:format cl:nil "observation_info[] observation_infos~%~%================================================================================~%MSG: tug_observers_msgs/observation_info~%Header header~%string type~%string resource~%observation[] observation~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: tug_observers_msgs/observation~%string observation_msg~%string verbose_observation_msg~%int32 observation~%int32 GENERAL_OK=0~%int32 GENERAL_ERROR=-1~%int32 NO_STATE_FITS=-2~%int32 NOT_AVAILABLE=-3~%int32 TIMEOUT=-4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <observer_info>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'observation_infos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <observer_info>))
  "Converts a ROS message object to a list"
  (cl:list 'observer_info
    (cl:cons ':observation_infos (observation_infos msg))
))
