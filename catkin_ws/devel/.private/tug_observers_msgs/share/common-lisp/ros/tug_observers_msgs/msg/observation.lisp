; Auto-generated. Do not edit!


(cl:in-package tug_observers_msgs-msg)


;//! \htmlinclude observation.msg.html

(cl:defclass <observation> (roslisp-msg-protocol:ros-message)
  ((observation_msg
    :reader observation_msg
    :initarg :observation_msg
    :type cl:string
    :initform "")
   (verbose_observation_msg
    :reader verbose_observation_msg
    :initarg :verbose_observation_msg
    :type cl:string
    :initform "")
   (observation
    :reader observation
    :initarg :observation
    :type cl:integer
    :initform 0))
)

(cl:defclass observation (<observation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <observation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'observation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tug_observers_msgs-msg:<observation> is deprecated: use tug_observers_msgs-msg:observation instead.")))

(cl:ensure-generic-function 'observation_msg-val :lambda-list '(m))
(cl:defmethod observation_msg-val ((m <observation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tug_observers_msgs-msg:observation_msg-val is deprecated.  Use tug_observers_msgs-msg:observation_msg instead.")
  (observation_msg m))

(cl:ensure-generic-function 'verbose_observation_msg-val :lambda-list '(m))
(cl:defmethod verbose_observation_msg-val ((m <observation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tug_observers_msgs-msg:verbose_observation_msg-val is deprecated.  Use tug_observers_msgs-msg:verbose_observation_msg instead.")
  (verbose_observation_msg m))

(cl:ensure-generic-function 'observation-val :lambda-list '(m))
(cl:defmethod observation-val ((m <observation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tug_observers_msgs-msg:observation-val is deprecated.  Use tug_observers_msgs-msg:observation instead.")
  (observation m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<observation>)))
    "Constants for message type '<observation>"
  '((:GENERAL_OK . 0)
    (:GENERAL_ERROR . -1)
    (:NO_STATE_FITS . -2)
    (:NOT_AVAILABLE . -3)
    (:TIMEOUT . -4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'observation)))
    "Constants for message type 'observation"
  '((:GENERAL_OK . 0)
    (:GENERAL_ERROR . -1)
    (:NO_STATE_FITS . -2)
    (:NOT_AVAILABLE . -3)
    (:TIMEOUT . -4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <observation>) ostream)
  "Serializes a message object of type '<observation>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'observation_msg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'observation_msg))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'verbose_observation_msg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'verbose_observation_msg))
  (cl:let* ((signed (cl:slot-value msg 'observation)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <observation>) istream)
  "Deserializes a message object of type '<observation>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'observation_msg) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'observation_msg) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'verbose_observation_msg) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'verbose_observation_msg) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'observation) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<observation>)))
  "Returns string type for a message object of type '<observation>"
  "tug_observers_msgs/observation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'observation)))
  "Returns string type for a message object of type 'observation"
  "tug_observers_msgs/observation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<observation>)))
  "Returns md5sum for a message object of type '<observation>"
  "176310c5b8642d2cf705c70f2da7bc39")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'observation)))
  "Returns md5sum for a message object of type 'observation"
  "176310c5b8642d2cf705c70f2da7bc39")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<observation>)))
  "Returns full string definition for message of type '<observation>"
  (cl:format cl:nil "string observation_msg~%string verbose_observation_msg~%int32 observation~%int32 GENERAL_OK=0~%int32 GENERAL_ERROR=-1~%int32 NO_STATE_FITS=-2~%int32 NOT_AVAILABLE=-3~%int32 TIMEOUT=-4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'observation)))
  "Returns full string definition for message of type 'observation"
  (cl:format cl:nil "string observation_msg~%string verbose_observation_msg~%int32 observation~%int32 GENERAL_OK=0~%int32 GENERAL_ERROR=-1~%int32 NO_STATE_FITS=-2~%int32 NOT_AVAILABLE=-3~%int32 TIMEOUT=-4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <observation>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'observation_msg))
     4 (cl:length (cl:slot-value msg 'verbose_observation_msg))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <observation>))
  "Converts a ROS message object to a list"
  (cl:list 'observation
    (cl:cons ':observation_msg (observation_msg msg))
    (cl:cons ':verbose_observation_msg (verbose_observation_msg msg))
    (cl:cons ':observation (observation msg))
))
