; Auto-generated. Do not edit!


(cl:in-package tug_diagnosis_msgs-srv)


;//! \htmlinclude DiagnosisConfiguration-request.msg.html

(cl:defclass <DiagnosisConfiguration-request> (roslisp-msg-protocol:ros-message)
  ((config
    :reader config
    :initarg :config
    :type tug_diagnosis_msgs-msg:configuration
    :initform (cl:make-instance 'tug_diagnosis_msgs-msg:configuration))
   (action
    :reader action
    :initarg :action
    :type cl:integer
    :initform 0))
)

(cl:defclass DiagnosisConfiguration-request (<DiagnosisConfiguration-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DiagnosisConfiguration-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DiagnosisConfiguration-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tug_diagnosis_msgs-srv:<DiagnosisConfiguration-request> is deprecated: use tug_diagnosis_msgs-srv:DiagnosisConfiguration-request instead.")))

(cl:ensure-generic-function 'config-val :lambda-list '(m))
(cl:defmethod config-val ((m <DiagnosisConfiguration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tug_diagnosis_msgs-srv:config-val is deprecated.  Use tug_diagnosis_msgs-srv:config instead.")
  (config m))

(cl:ensure-generic-function 'action-val :lambda-list '(m))
(cl:defmethod action-val ((m <DiagnosisConfiguration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tug_diagnosis_msgs-srv:action-val is deprecated.  Use tug_diagnosis_msgs-srv:action instead.")
  (action m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<DiagnosisConfiguration-request>)))
    "Constants for message type '<DiagnosisConfiguration-request>"
  '((:ADD . 1)
    (:REMOVE . 2)
    (:SET . 3)
    (:UPDATE . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'DiagnosisConfiguration-request)))
    "Constants for message type 'DiagnosisConfiguration-request"
  '((:ADD . 1)
    (:REMOVE . 2)
    (:SET . 3)
    (:UPDATE . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DiagnosisConfiguration-request>) ostream)
  "Serializes a message object of type '<DiagnosisConfiguration-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'config) ostream)
  (cl:let* ((signed (cl:slot-value msg 'action)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DiagnosisConfiguration-request>) istream)
  "Deserializes a message object of type '<DiagnosisConfiguration-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'config) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'action) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DiagnosisConfiguration-request>)))
  "Returns string type for a service object of type '<DiagnosisConfiguration-request>"
  "tug_diagnosis_msgs/DiagnosisConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DiagnosisConfiguration-request)))
  "Returns string type for a service object of type 'DiagnosisConfiguration-request"
  "tug_diagnosis_msgs/DiagnosisConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DiagnosisConfiguration-request>)))
  "Returns md5sum for a message object of type '<DiagnosisConfiguration-request>"
  "43fad42cb6cdc7f9a57a24dd1daa5334")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DiagnosisConfiguration-request)))
  "Returns md5sum for a message object of type 'DiagnosisConfiguration-request"
  "43fad42cb6cdc7f9a57a24dd1daa5334")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DiagnosisConfiguration-request>)))
  "Returns full string definition for message of type '<DiagnosisConfiguration-request>"
  (cl:format cl:nil "~%configuration config~%~%~%int32 action~%int32 ADD=1~%int32 REMOVE=2~%int32 SET=3~%int32 UPDATE=4~%~%~%================================================================================~%MSG: tug_diagnosis_msgs/configuration~%node_configuration[] nodes~%observer_configuration[] observers~%~%================================================================================~%MSG: tug_diagnosis_msgs/node_configuration~%string name~%string[] sub_topic~%string[] pub_topic~%~%================================================================================~%MSG: tug_diagnosis_msgs/observer_configuration~%string type~%string[] resource~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DiagnosisConfiguration-request)))
  "Returns full string definition for message of type 'DiagnosisConfiguration-request"
  (cl:format cl:nil "~%configuration config~%~%~%int32 action~%int32 ADD=1~%int32 REMOVE=2~%int32 SET=3~%int32 UPDATE=4~%~%~%================================================================================~%MSG: tug_diagnosis_msgs/configuration~%node_configuration[] nodes~%observer_configuration[] observers~%~%================================================================================~%MSG: tug_diagnosis_msgs/node_configuration~%string name~%string[] sub_topic~%string[] pub_topic~%~%================================================================================~%MSG: tug_diagnosis_msgs/observer_configuration~%string type~%string[] resource~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DiagnosisConfiguration-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'config))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DiagnosisConfiguration-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DiagnosisConfiguration-request
    (cl:cons ':config (config msg))
    (cl:cons ':action (action msg))
))
;//! \htmlinclude DiagnosisConfiguration-response.msg.html

(cl:defclass <DiagnosisConfiguration-response> (roslisp-msg-protocol:ros-message)
  ((errorcode
    :reader errorcode
    :initarg :errorcode
    :type cl:integer
    :initform 0)
   (error_msg
    :reader error_msg
    :initarg :error_msg
    :type cl:string
    :initform ""))
)

(cl:defclass DiagnosisConfiguration-response (<DiagnosisConfiguration-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DiagnosisConfiguration-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DiagnosisConfiguration-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tug_diagnosis_msgs-srv:<DiagnosisConfiguration-response> is deprecated: use tug_diagnosis_msgs-srv:DiagnosisConfiguration-response instead.")))

(cl:ensure-generic-function 'errorcode-val :lambda-list '(m))
(cl:defmethod errorcode-val ((m <DiagnosisConfiguration-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tug_diagnosis_msgs-srv:errorcode-val is deprecated.  Use tug_diagnosis_msgs-srv:errorcode instead.")
  (errorcode m))

(cl:ensure-generic-function 'error_msg-val :lambda-list '(m))
(cl:defmethod error_msg-val ((m <DiagnosisConfiguration-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tug_diagnosis_msgs-srv:error_msg-val is deprecated.  Use tug_diagnosis_msgs-srv:error_msg instead.")
  (error_msg m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<DiagnosisConfiguration-response>)))
    "Constants for message type '<DiagnosisConfiguration-response>"
  '((:NO_ERROR . 0)
    (:GENERAL_ERROR . -1)
    (:CONFIG_INVALID . -2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'DiagnosisConfiguration-response)))
    "Constants for message type 'DiagnosisConfiguration-response"
  '((:NO_ERROR . 0)
    (:GENERAL_ERROR . -1)
    (:CONFIG_INVALID . -2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DiagnosisConfiguration-response>) ostream)
  "Serializes a message object of type '<DiagnosisConfiguration-response>"
  (cl:let* ((signed (cl:slot-value msg 'errorcode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'error_msg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'error_msg))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DiagnosisConfiguration-response>) istream)
  "Deserializes a message object of type '<DiagnosisConfiguration-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'errorcode) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'error_msg) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'error_msg) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DiagnosisConfiguration-response>)))
  "Returns string type for a service object of type '<DiagnosisConfiguration-response>"
  "tug_diagnosis_msgs/DiagnosisConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DiagnosisConfiguration-response)))
  "Returns string type for a service object of type 'DiagnosisConfiguration-response"
  "tug_diagnosis_msgs/DiagnosisConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DiagnosisConfiguration-response>)))
  "Returns md5sum for a message object of type '<DiagnosisConfiguration-response>"
  "43fad42cb6cdc7f9a57a24dd1daa5334")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DiagnosisConfiguration-response)))
  "Returns md5sum for a message object of type 'DiagnosisConfiguration-response"
  "43fad42cb6cdc7f9a57a24dd1daa5334")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DiagnosisConfiguration-response>)))
  "Returns full string definition for message of type '<DiagnosisConfiguration-response>"
  (cl:format cl:nil "~%~%int32 errorcode~%string error_msg~%int32 NO_ERROR=0~%int32 GENERAL_ERROR=-1~%int32 CONFIG_INVALID=-2~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DiagnosisConfiguration-response)))
  "Returns full string definition for message of type 'DiagnosisConfiguration-response"
  (cl:format cl:nil "~%~%int32 errorcode~%string error_msg~%int32 NO_ERROR=0~%int32 GENERAL_ERROR=-1~%int32 CONFIG_INVALID=-2~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DiagnosisConfiguration-response>))
  (cl:+ 0
     4
     4 (cl:length (cl:slot-value msg 'error_msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DiagnosisConfiguration-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DiagnosisConfiguration-response
    (cl:cons ':errorcode (errorcode msg))
    (cl:cons ':error_msg (error_msg msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DiagnosisConfiguration)))
  'DiagnosisConfiguration-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DiagnosisConfiguration)))
  'DiagnosisConfiguration-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DiagnosisConfiguration)))
  "Returns string type for a service object of type '<DiagnosisConfiguration>"
  "tug_diagnosis_msgs/DiagnosisConfiguration")