; Auto-generated. Do not edit!


(cl:in-package tug_resource_monitor-srv)


;//! \htmlinclude NodesInfo-request.msg.html

(cl:defclass <NodesInfo-request> (roslisp-msg-protocol:ros-message)
  ((node_names
    :reader node_names
    :initarg :node_names
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass NodesInfo-request (<NodesInfo-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NodesInfo-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NodesInfo-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tug_resource_monitor-srv:<NodesInfo-request> is deprecated: use tug_resource_monitor-srv:NodesInfo-request instead.")))

(cl:ensure-generic-function 'node_names-val :lambda-list '(m))
(cl:defmethod node_names-val ((m <NodesInfo-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tug_resource_monitor-srv:node_names-val is deprecated.  Use tug_resource_monitor-srv:node_names instead.")
  (node_names m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NodesInfo-request>) ostream)
  "Serializes a message object of type '<NodesInfo-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'node_names))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'node_names))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NodesInfo-request>) istream)
  "Deserializes a message object of type '<NodesInfo-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'node_names) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'node_names)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NodesInfo-request>)))
  "Returns string type for a service object of type '<NodesInfo-request>"
  "tug_resource_monitor/NodesInfoRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NodesInfo-request)))
  "Returns string type for a service object of type 'NodesInfo-request"
  "tug_resource_monitor/NodesInfoRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NodesInfo-request>)))
  "Returns md5sum for a message object of type '<NodesInfo-request>"
  "42c13fd17030d24481c0409739ac5ae7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NodesInfo-request)))
  "Returns md5sum for a message object of type 'NodesInfo-request"
  "42c13fd17030d24481c0409739ac5ae7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NodesInfo-request>)))
  "Returns full string definition for message of type '<NodesInfo-request>"
  (cl:format cl:nil "string[] node_names~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NodesInfo-request)))
  "Returns full string definition for message of type 'NodesInfo-request"
  (cl:format cl:nil "string[] node_names~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NodesInfo-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'node_names) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NodesInfo-request>))
  "Converts a ROS message object to a list"
  (cl:list 'NodesInfo-request
    (cl:cons ':node_names (node_names msg))
))
;//! \htmlinclude NodesInfo-response.msg.html

(cl:defclass <NodesInfo-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass NodesInfo-response (<NodesInfo-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NodesInfo-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NodesInfo-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tug_resource_monitor-srv:<NodesInfo-response> is deprecated: use tug_resource_monitor-srv:NodesInfo-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NodesInfo-response>) ostream)
  "Serializes a message object of type '<NodesInfo-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NodesInfo-response>) istream)
  "Deserializes a message object of type '<NodesInfo-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NodesInfo-response>)))
  "Returns string type for a service object of type '<NodesInfo-response>"
  "tug_resource_monitor/NodesInfoResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NodesInfo-response)))
  "Returns string type for a service object of type 'NodesInfo-response"
  "tug_resource_monitor/NodesInfoResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NodesInfo-response>)))
  "Returns md5sum for a message object of type '<NodesInfo-response>"
  "42c13fd17030d24481c0409739ac5ae7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NodesInfo-response)))
  "Returns md5sum for a message object of type 'NodesInfo-response"
  "42c13fd17030d24481c0409739ac5ae7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NodesInfo-response>)))
  "Returns full string definition for message of type '<NodesInfo-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NodesInfo-response)))
  "Returns full string definition for message of type 'NodesInfo-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NodesInfo-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NodesInfo-response>))
  "Converts a ROS message object to a list"
  (cl:list 'NodesInfo-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'NodesInfo)))
  'NodesInfo-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'NodesInfo)))
  'NodesInfo-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NodesInfo)))
  "Returns string type for a service object of type '<NodesInfo>"
  "tug_resource_monitor/NodesInfo")