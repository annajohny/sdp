; Auto-generated. Do not edit!


(cl:in-package tug_diagnosis_msgs-msg)


;//! \htmlinclude observer_configuration.msg.html

(cl:defclass <observer_configuration> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:string
    :initform "")
   (resource
    :reader resource
    :initarg :resource
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass observer_configuration (<observer_configuration>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <observer_configuration>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'observer_configuration)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tug_diagnosis_msgs-msg:<observer_configuration> is deprecated: use tug_diagnosis_msgs-msg:observer_configuration instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <observer_configuration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tug_diagnosis_msgs-msg:type-val is deprecated.  Use tug_diagnosis_msgs-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'resource-val :lambda-list '(m))
(cl:defmethod resource-val ((m <observer_configuration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tug_diagnosis_msgs-msg:resource-val is deprecated.  Use tug_diagnosis_msgs-msg:resource instead.")
  (resource m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <observer_configuration>) ostream)
  "Serializes a message object of type '<observer_configuration>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'type))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'resource))))
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
   (cl:slot-value msg 'resource))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <observer_configuration>) istream)
  "Deserializes a message object of type '<observer_configuration>"
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
  (cl:setf (cl:slot-value msg 'resource) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'resource)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<observer_configuration>)))
  "Returns string type for a message object of type '<observer_configuration>"
  "tug_diagnosis_msgs/observer_configuration")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'observer_configuration)))
  "Returns string type for a message object of type 'observer_configuration"
  "tug_diagnosis_msgs/observer_configuration")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<observer_configuration>)))
  "Returns md5sum for a message object of type '<observer_configuration>"
  "a2f5ebb423c16ad934f5d2cae333e2df")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'observer_configuration)))
  "Returns md5sum for a message object of type 'observer_configuration"
  "a2f5ebb423c16ad934f5d2cae333e2df")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<observer_configuration>)))
  "Returns full string definition for message of type '<observer_configuration>"
  (cl:format cl:nil "string type~%string[] resource~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'observer_configuration)))
  "Returns full string definition for message of type 'observer_configuration"
  (cl:format cl:nil "string type~%string[] resource~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <observer_configuration>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'type))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'resource) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <observer_configuration>))
  "Converts a ROS message object to a list"
  (cl:list 'observer_configuration
    (cl:cons ':type (type msg))
    (cl:cons ':resource (resource msg))
))
