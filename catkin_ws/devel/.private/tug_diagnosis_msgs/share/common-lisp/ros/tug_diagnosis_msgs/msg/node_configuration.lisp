; Auto-generated. Do not edit!


(cl:in-package tug_diagnosis_msgs-msg)


;//! \htmlinclude node_configuration.msg.html

(cl:defclass <node_configuration> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (sub_topic
    :reader sub_topic
    :initarg :sub_topic
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (pub_topic
    :reader pub_topic
    :initarg :pub_topic
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass node_configuration (<node_configuration>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <node_configuration>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'node_configuration)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tug_diagnosis_msgs-msg:<node_configuration> is deprecated: use tug_diagnosis_msgs-msg:node_configuration instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <node_configuration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tug_diagnosis_msgs-msg:name-val is deprecated.  Use tug_diagnosis_msgs-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'sub_topic-val :lambda-list '(m))
(cl:defmethod sub_topic-val ((m <node_configuration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tug_diagnosis_msgs-msg:sub_topic-val is deprecated.  Use tug_diagnosis_msgs-msg:sub_topic instead.")
  (sub_topic m))

(cl:ensure-generic-function 'pub_topic-val :lambda-list '(m))
(cl:defmethod pub_topic-val ((m <node_configuration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tug_diagnosis_msgs-msg:pub_topic-val is deprecated.  Use tug_diagnosis_msgs-msg:pub_topic instead.")
  (pub_topic m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <node_configuration>) ostream)
  "Serializes a message object of type '<node_configuration>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'sub_topic))))
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
   (cl:slot-value msg 'sub_topic))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'pub_topic))))
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
   (cl:slot-value msg 'pub_topic))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <node_configuration>) istream)
  "Deserializes a message object of type '<node_configuration>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'sub_topic) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'sub_topic)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'pub_topic) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'pub_topic)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<node_configuration>)))
  "Returns string type for a message object of type '<node_configuration>"
  "tug_diagnosis_msgs/node_configuration")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'node_configuration)))
  "Returns string type for a message object of type 'node_configuration"
  "tug_diagnosis_msgs/node_configuration")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<node_configuration>)))
  "Returns md5sum for a message object of type '<node_configuration>"
  "d437a072fcff0973a4326ed765667e76")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'node_configuration)))
  "Returns md5sum for a message object of type 'node_configuration"
  "d437a072fcff0973a4326ed765667e76")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<node_configuration>)))
  "Returns full string definition for message of type '<node_configuration>"
  (cl:format cl:nil "string name~%string[] sub_topic~%string[] pub_topic~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'node_configuration)))
  "Returns full string definition for message of type 'node_configuration"
  (cl:format cl:nil "string name~%string[] sub_topic~%string[] pub_topic~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <node_configuration>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'sub_topic) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'pub_topic) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <node_configuration>))
  "Converts a ROS message object to a list"
  (cl:list 'node_configuration
    (cl:cons ':name (name msg))
    (cl:cons ':sub_topic (sub_topic msg))
    (cl:cons ':pub_topic (pub_topic msg))
))
