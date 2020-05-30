;; Auto-generated. Do not edit!


(when (boundp 'tug_diagnosis_msgs::diagnosis_set)
  (if (not (find-package "TUG_DIAGNOSIS_MSGS"))
    (make-package "TUG_DIAGNOSIS_MSGS"))
  (shadow 'diagnosis_set (find-package "TUG_DIAGNOSIS_MSGS")))
(unless (find-package "TUG_DIAGNOSIS_MSGS::DIAGNOSIS_SET")
  (make-package "TUG_DIAGNOSIS_MSGS::DIAGNOSIS_SET"))

(in-package "ROS")
;;//! \htmlinclude diagnosis_set.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass tug_diagnosis_msgs::diagnosis_set
  :super ros::object
  :slots (_header _type _diagnoses ))

(defmethod tug_diagnosis_msgs::diagnosis_set
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:type __type) "")
    ((:diagnoses __diagnoses) (let (r) (dotimes (i 0) (push (instance tug_diagnosis_msgs::diagnosis :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _type (string __type))
   (setq _diagnoses __diagnoses)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:diagnoses
   (&rest __diagnoses)
   (if (keywordp (car __diagnoses))
       (send* _diagnoses __diagnoses)
     (progn
       (if __diagnoses (setq _diagnoses (car __diagnoses)))
       _diagnoses)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _type
    4 (length _type)
    ;; tug_diagnosis_msgs/diagnosis[] _diagnoses
    (apply #'+ (send-all _diagnoses :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _type
       (write-long (length _type) s) (princ _type s)
     ;; tug_diagnosis_msgs/diagnosis[] _diagnoses
     (write-long (length _diagnoses) s)
     (dolist (elem _diagnoses)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; tug_diagnosis_msgs/diagnosis[] _diagnoses
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _diagnoses (let (r) (dotimes (i n) (push (instance tug_diagnosis_msgs::diagnosis :init) r)) r))
     (dolist (elem- _diagnoses)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get tug_diagnosis_msgs::diagnosis_set :md5sum-) "1e8ee6a1d3f192b8969b6d2785e6690b")
(setf (get tug_diagnosis_msgs::diagnosis_set :datatype-) "tug_diagnosis_msgs/diagnosis_set")
(setf (get tug_diagnosis_msgs::diagnosis_set :definition-)
      "Header header
string type
diagnosis[] diagnoses

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: tug_diagnosis_msgs/diagnosis
resource_mode_assignement[] diagnosis

================================================================================
MSG: tug_diagnosis_msgs/resource_mode_assignement
string resource
string mode_msg
int32 mode
int32 GENERAL_OK=0
int32 GENERAL_ERROR=-1

")



(provide :tug_diagnosis_msgs/diagnosis_set "1e8ee6a1d3f192b8969b6d2785e6690b")


