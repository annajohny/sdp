;; Auto-generated. Do not edit!


(when (boundp 'tug_resource_monitor::NodeInfoArray)
  (if (not (find-package "TUG_RESOURCE_MONITOR"))
    (make-package "TUG_RESOURCE_MONITOR"))
  (shadow 'NodeInfoArray (find-package "TUG_RESOURCE_MONITOR")))
(unless (find-package "TUG_RESOURCE_MONITOR::NODEINFOARRAY")
  (make-package "TUG_RESOURCE_MONITOR::NODEINFOARRAY"))

(in-package "ROS")
;;//! \htmlinclude NodeInfoArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass tug_resource_monitor::NodeInfoArray
  :super ros::object
  :slots (_header _data ))

(defmethod tug_resource_monitor::NodeInfoArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:data __data) (let (r) (dotimes (i 0) (push (instance tug_resource_monitor::NodeInfo :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _data __data)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:data
   (&rest __data)
   (if (keywordp (car __data))
       (send* _data __data)
     (progn
       (if __data (setq _data (car __data)))
       _data)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; tug_resource_monitor/NodeInfo[] _data
    (apply #'+ (send-all _data :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; tug_resource_monitor/NodeInfo[] _data
     (write-long (length _data) s)
     (dolist (elem _data)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; tug_resource_monitor/NodeInfo[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (let (r) (dotimes (i n) (push (instance tug_resource_monitor::NodeInfo :init) r)) r))
     (dolist (elem- _data)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get tug_resource_monitor::NodeInfoArray :md5sum-) "36980c6bc4abe5c7f2d787bbf2d3e856")
(setf (get tug_resource_monitor::NodeInfoArray :datatype-) "tug_resource_monitor/NodeInfoArray")
(setf (get tug_resource_monitor::NodeInfoArray :definition-)
      "Header header
NodeInfo[] data

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
MSG: tug_resource_monitor/NodeInfo
string name
uint32 pid
string hostname
float32 cpu
uint64 memory
uint32 error
uint32 NO_ERROR=0
uint32 ERROR_PID_NOT_FOUND=1

")



(provide :tug_resource_monitor/NodeInfoArray "36980c6bc4abe5c7f2d787bbf2d3e856")


