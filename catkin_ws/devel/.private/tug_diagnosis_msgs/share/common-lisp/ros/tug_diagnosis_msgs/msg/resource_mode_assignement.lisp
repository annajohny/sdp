; Auto-generated. Do not edit!


(cl:in-package tug_diagnosis_msgs-msg)


;//! \htmlinclude resource_mode_assignement.msg.html

(cl:defclass <resource_mode_assignement> (roslisp-msg-protocol:ros-message)
  ((resource
    :reader resource
    :initarg :resource
    :type cl:string
    :initform "")
   (mode_msg
    :reader mode_msg
    :initarg :mode_msg
    :type cl:string
    :initform "")
   (mode
    :reader mode
    :initarg :mode
    :type cl:integer
    :initform 0))
)

(cl:defclass resource_mode_assignement (<resource_mode_assignement>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <resource_mode_assignement>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'resource_mode_assignement)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tug_diagnosis_msgs-msg:<resource_mode_assignement> is deprecated: use tug_diagnosis_msgs-msg:resource_mode_assignement instead.")))

(cl:ensure-generic-function 'resource-val :lambda-list '(m))
(cl:defmethod resource-val ((m <resource_mode_assignement>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tug_diagnosis_msgs-msg:resource-val is deprecated.  Use tug_diagnosis_msgs-msg:resource instead.")
  (resource m))

(cl:ensure-generic-function 'mode_msg-val :lambda-list '(m))
(cl:defmethod mode_msg-val ((m <resource_mode_assignement>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tug_diagnosis_msgs-msg:mode_msg-val is deprecated.  Use tug_diagnosis_msgs-msg:mode_msg instead.")
  (mode_msg m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <resource_mode_assignement>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tug_diagnosis_msgs-msg:mode-val is deprecated.  Use tug_diagnosis_msgs-msg:mode instead.")
  (mode m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<resource_mode_assignement>)))
    "Constants for message type '<resource_mode_assignement>"
  '((:GENERAL_OK . 0)
    (:GENERAL_ERROR . -1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'resource_mode_assignement)))
    "Constants for message type 'resource_mode_assignement"
  '((:GENERAL_OK . 0)
    (:GENERAL_ERROR . -1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <resource_mode_assignement>) ostream)
  "Serializes a message object of type '<resource_mode_assignement>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'resource))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'resource))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'mode_msg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'mode_msg))
  (cl:let* ((signed (cl:slot-value msg 'mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <resource_mode_assignement>) istream)
  "Deserializes a message object of type '<resource_mode_assignement>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'resource) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'resource) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode_msg) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'mode_msg) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<resource_mode_assignement>)))
  "Returns string type for a message object of type '<resource_mode_assignement>"
  "tug_diagnosis_msgs/resource_mode_assignement")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'resource_mode_assignement)))
  "Returns string type for a message object of type 'resource_mode_assignement"
  "tug_diagnosis_msgs/resource_mode_assignement")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<resource_mode_assignement>)))
  "Returns md5sum for a message object of type '<resource_mode_assignement>"
  "3bb0fd0cf9c222fef578a66ea18f38c0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'resource_mode_assignement)))
  "Returns md5sum for a message object of type 'resource_mode_assignement"
  "3bb0fd0cf9c222fef578a66ea18f38c0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<resource_mode_assignement>)))
  "Returns full string definition for message of type '<resource_mode_assignement>"
  (cl:format cl:nil "string resource~%string mode_msg~%int32 mode~%int32 GENERAL_OK=0~%int32 GENERAL_ERROR=-1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'resource_mode_assignement)))
  "Returns full string definition for message of type 'resource_mode_assignement"
  (cl:format cl:nil "string resource~%string mode_msg~%int32 mode~%int32 GENERAL_OK=0~%int32 GENERAL_ERROR=-1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <resource_mode_assignement>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'resource))
     4 (cl:length (cl:slot-value msg 'mode_msg))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <resource_mode_assignement>))
  "Converts a ROS message object to a list"
  (cl:list 'resource_mode_assignement
    (cl:cons ':resource (resource msg))
    (cl:cons ':mode_msg (mode_msg msg))
    (cl:cons ':mode (mode msg))
))
