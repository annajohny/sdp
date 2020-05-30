; Auto-generated. Do not edit!


(cl:in-package tug_resource_monitor-msg)


;//! \htmlinclude NodeInfo.msg.html

(cl:defclass <NodeInfo> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (pid
    :reader pid
    :initarg :pid
    :type cl:integer
    :initform 0)
   (hostname
    :reader hostname
    :initarg :hostname
    :type cl:string
    :initform "")
   (cpu
    :reader cpu
    :initarg :cpu
    :type cl:float
    :initform 0.0)
   (memory
    :reader memory
    :initarg :memory
    :type cl:integer
    :initform 0)
   (error
    :reader error
    :initarg :error
    :type cl:integer
    :initform 0))
)

(cl:defclass NodeInfo (<NodeInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NodeInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NodeInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tug_resource_monitor-msg:<NodeInfo> is deprecated: use tug_resource_monitor-msg:NodeInfo instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <NodeInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tug_resource_monitor-msg:name-val is deprecated.  Use tug_resource_monitor-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'pid-val :lambda-list '(m))
(cl:defmethod pid-val ((m <NodeInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tug_resource_monitor-msg:pid-val is deprecated.  Use tug_resource_monitor-msg:pid instead.")
  (pid m))

(cl:ensure-generic-function 'hostname-val :lambda-list '(m))
(cl:defmethod hostname-val ((m <NodeInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tug_resource_monitor-msg:hostname-val is deprecated.  Use tug_resource_monitor-msg:hostname instead.")
  (hostname m))

(cl:ensure-generic-function 'cpu-val :lambda-list '(m))
(cl:defmethod cpu-val ((m <NodeInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tug_resource_monitor-msg:cpu-val is deprecated.  Use tug_resource_monitor-msg:cpu instead.")
  (cpu m))

(cl:ensure-generic-function 'memory-val :lambda-list '(m))
(cl:defmethod memory-val ((m <NodeInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tug_resource_monitor-msg:memory-val is deprecated.  Use tug_resource_monitor-msg:memory instead.")
  (memory m))

(cl:ensure-generic-function 'error-val :lambda-list '(m))
(cl:defmethod error-val ((m <NodeInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tug_resource_monitor-msg:error-val is deprecated.  Use tug_resource_monitor-msg:error instead.")
  (error m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<NodeInfo>)))
    "Constants for message type '<NodeInfo>"
  '((:NO_ERROR . 0)
    (:ERROR_PID_NOT_FOUND . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'NodeInfo)))
    "Constants for message type 'NodeInfo"
  '((:NO_ERROR . 0)
    (:ERROR_PID_NOT_FOUND . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NodeInfo>) ostream)
  "Serializes a message object of type '<NodeInfo>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'pid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'pid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'pid)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'hostname))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'hostname))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cpu))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'memory)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'memory)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'memory)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'memory)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'memory)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'memory)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'memory)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'memory)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'error)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'error)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'error)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'error)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NodeInfo>) istream)
  "Deserializes a message object of type '<NodeInfo>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'pid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'pid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'pid)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'hostname) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'hostname) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cpu) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'memory)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'memory)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'memory)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'memory)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'memory)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'memory)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'memory)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'memory)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'error)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'error)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'error)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'error)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NodeInfo>)))
  "Returns string type for a message object of type '<NodeInfo>"
  "tug_resource_monitor/NodeInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NodeInfo)))
  "Returns string type for a message object of type 'NodeInfo"
  "tug_resource_monitor/NodeInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NodeInfo>)))
  "Returns md5sum for a message object of type '<NodeInfo>"
  "b3df41a0cc3ca1f8f984ebc9825c7a08")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NodeInfo)))
  "Returns md5sum for a message object of type 'NodeInfo"
  "b3df41a0cc3ca1f8f984ebc9825c7a08")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NodeInfo>)))
  "Returns full string definition for message of type '<NodeInfo>"
  (cl:format cl:nil "string name~%uint32 pid~%string hostname~%float32 cpu~%uint64 memory~%uint32 error~%uint32 NO_ERROR=0~%uint32 ERROR_PID_NOT_FOUND=1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NodeInfo)))
  "Returns full string definition for message of type 'NodeInfo"
  (cl:format cl:nil "string name~%uint32 pid~%string hostname~%float32 cpu~%uint64 memory~%uint32 error~%uint32 NO_ERROR=0~%uint32 ERROR_PID_NOT_FOUND=1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NodeInfo>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     4
     4 (cl:length (cl:slot-value msg 'hostname))
     4
     8
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NodeInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'NodeInfo
    (cl:cons ':name (name msg))
    (cl:cons ':pid (pid msg))
    (cl:cons ':hostname (hostname msg))
    (cl:cons ':cpu (cpu msg))
    (cl:cons ':memory (memory msg))
    (cl:cons ':error (error msg))
))
