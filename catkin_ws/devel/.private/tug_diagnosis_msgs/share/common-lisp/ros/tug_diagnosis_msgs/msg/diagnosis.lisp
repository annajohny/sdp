; Auto-generated. Do not edit!


(cl:in-package tug_diagnosis_msgs-msg)


;//! \htmlinclude diagnosis.msg.html

(cl:defclass <diagnosis> (roslisp-msg-protocol:ros-message)
  ((diagnosis
    :reader diagnosis
    :initarg :diagnosis
    :type (cl:vector tug_diagnosis_msgs-msg:resource_mode_assignement)
   :initform (cl:make-array 0 :element-type 'tug_diagnosis_msgs-msg:resource_mode_assignement :initial-element (cl:make-instance 'tug_diagnosis_msgs-msg:resource_mode_assignement))))
)

(cl:defclass diagnosis (<diagnosis>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <diagnosis>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'diagnosis)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tug_diagnosis_msgs-msg:<diagnosis> is deprecated: use tug_diagnosis_msgs-msg:diagnosis instead.")))

(cl:ensure-generic-function 'diagnosis-val :lambda-list '(m))
(cl:defmethod diagnosis-val ((m <diagnosis>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tug_diagnosis_msgs-msg:diagnosis-val is deprecated.  Use tug_diagnosis_msgs-msg:diagnosis instead.")
  (diagnosis m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <diagnosis>) ostream)
  "Serializes a message object of type '<diagnosis>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'diagnosis))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'diagnosis))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <diagnosis>) istream)
  "Deserializes a message object of type '<diagnosis>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'diagnosis) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'diagnosis)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'tug_diagnosis_msgs-msg:resource_mode_assignement))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<diagnosis>)))
  "Returns string type for a message object of type '<diagnosis>"
  "tug_diagnosis_msgs/diagnosis")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'diagnosis)))
  "Returns string type for a message object of type 'diagnosis"
  "tug_diagnosis_msgs/diagnosis")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<diagnosis>)))
  "Returns md5sum for a message object of type '<diagnosis>"
  "52161f8a0d1740c6fc459d4f815e3e6d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'diagnosis)))
  "Returns md5sum for a message object of type 'diagnosis"
  "52161f8a0d1740c6fc459d4f815e3e6d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<diagnosis>)))
  "Returns full string definition for message of type '<diagnosis>"
  (cl:format cl:nil "resource_mode_assignement[] diagnosis~%~%================================================================================~%MSG: tug_diagnosis_msgs/resource_mode_assignement~%string resource~%string mode_msg~%int32 mode~%int32 GENERAL_OK=0~%int32 GENERAL_ERROR=-1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'diagnosis)))
  "Returns full string definition for message of type 'diagnosis"
  (cl:format cl:nil "resource_mode_assignement[] diagnosis~%~%================================================================================~%MSG: tug_diagnosis_msgs/resource_mode_assignement~%string resource~%string mode_msg~%int32 mode~%int32 GENERAL_OK=0~%int32 GENERAL_ERROR=-1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <diagnosis>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'diagnosis) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <diagnosis>))
  "Converts a ROS message object to a list"
  (cl:list 'diagnosis
    (cl:cons ':diagnosis (diagnosis msg))
))
