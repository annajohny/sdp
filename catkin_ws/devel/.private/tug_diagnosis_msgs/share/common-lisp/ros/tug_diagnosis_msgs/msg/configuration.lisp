; Auto-generated. Do not edit!


(cl:in-package tug_diagnosis_msgs-msg)


;//! \htmlinclude configuration.msg.html

(cl:defclass <configuration> (roslisp-msg-protocol:ros-message)
  ((nodes
    :reader nodes
    :initarg :nodes
    :type (cl:vector tug_diagnosis_msgs-msg:node_configuration)
   :initform (cl:make-array 0 :element-type 'tug_diagnosis_msgs-msg:node_configuration :initial-element (cl:make-instance 'tug_diagnosis_msgs-msg:node_configuration)))
   (observers
    :reader observers
    :initarg :observers
    :type (cl:vector tug_diagnosis_msgs-msg:observer_configuration)
   :initform (cl:make-array 0 :element-type 'tug_diagnosis_msgs-msg:observer_configuration :initial-element (cl:make-instance 'tug_diagnosis_msgs-msg:observer_configuration))))
)

(cl:defclass configuration (<configuration>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <configuration>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'configuration)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tug_diagnosis_msgs-msg:<configuration> is deprecated: use tug_diagnosis_msgs-msg:configuration instead.")))

(cl:ensure-generic-function 'nodes-val :lambda-list '(m))
(cl:defmethod nodes-val ((m <configuration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tug_diagnosis_msgs-msg:nodes-val is deprecated.  Use tug_diagnosis_msgs-msg:nodes instead.")
  (nodes m))

(cl:ensure-generic-function 'observers-val :lambda-list '(m))
(cl:defmethod observers-val ((m <configuration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tug_diagnosis_msgs-msg:observers-val is deprecated.  Use tug_diagnosis_msgs-msg:observers instead.")
  (observers m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <configuration>) ostream)
  "Serializes a message object of type '<configuration>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'nodes))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'nodes))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'observers))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'observers))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <configuration>) istream)
  "Deserializes a message object of type '<configuration>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'nodes) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'nodes)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'tug_diagnosis_msgs-msg:node_configuration))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'observers) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'observers)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'tug_diagnosis_msgs-msg:observer_configuration))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<configuration>)))
  "Returns string type for a message object of type '<configuration>"
  "tug_diagnosis_msgs/configuration")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'configuration)))
  "Returns string type for a message object of type 'configuration"
  "tug_diagnosis_msgs/configuration")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<configuration>)))
  "Returns md5sum for a message object of type '<configuration>"
  "cadde33865d5cf9cef30c348d588d747")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'configuration)))
  "Returns md5sum for a message object of type 'configuration"
  "cadde33865d5cf9cef30c348d588d747")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<configuration>)))
  "Returns full string definition for message of type '<configuration>"
  (cl:format cl:nil "node_configuration[] nodes~%observer_configuration[] observers~%~%================================================================================~%MSG: tug_diagnosis_msgs/node_configuration~%string name~%string[] sub_topic~%string[] pub_topic~%~%================================================================================~%MSG: tug_diagnosis_msgs/observer_configuration~%string type~%string[] resource~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'configuration)))
  "Returns full string definition for message of type 'configuration"
  (cl:format cl:nil "node_configuration[] nodes~%observer_configuration[] observers~%~%================================================================================~%MSG: tug_diagnosis_msgs/node_configuration~%string name~%string[] sub_topic~%string[] pub_topic~%~%================================================================================~%MSG: tug_diagnosis_msgs/observer_configuration~%string type~%string[] resource~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <configuration>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'nodes) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'observers) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <configuration>))
  "Converts a ROS message object to a list"
  (cl:list 'configuration
    (cl:cons ':nodes (nodes msg))
    (cl:cons ':observers (observers msg))
))
