;; Auto-generated. Do not edit!


(when (boundp 'tug_observers_msgs::observer_info)
  (if (not (find-package "TUG_OBSERVERS_MSGS"))
    (make-package "TUG_OBSERVERS_MSGS"))
  (shadow 'observer_info (find-package "TUG_OBSERVERS_MSGS")))
(unless (find-package "TUG_OBSERVERS_MSGS::OBSERVER_INFO")
  (make-package "TUG_OBSERVERS_MSGS::OBSERVER_INFO"))

(in-package "ROS")
;;//! \htmlinclude observer_info.msg.html


(defclass tug_observers_msgs::observer_info
  :super ros::object
  :slots (_observation_infos ))

(defmethod tug_observers_msgs::observer_info
  (:init
   (&key
    ((:observation_infos __observation_infos) (let (r) (dotimes (i 0) (push (instance tug_observers_msgs::observation_info :init) r)) r))
    )
   (send-super :init)
   (setq _observation_infos __observation_infos)
   self)
  (:observation_infos
   (&rest __observation_infos)
   (if (keywordp (car __observation_infos))
       (send* _observation_infos __observation_infos)
     (progn
       (if __observation_infos (setq _observation_infos (car __observation_infos)))
       _observation_infos)))
  (:serialization-length
   ()
   (+
    ;; tug_observers_msgs/observation_info[] _observation_infos
    (apply #'+ (send-all _observation_infos :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; tug_observers_msgs/observation_info[] _observation_infos
     (write-long (length _observation_infos) s)
     (dolist (elem _observation_infos)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; tug_observers_msgs/observation_info[] _observation_infos
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _observation_infos (let (r) (dotimes (i n) (push (instance tug_observers_msgs::observation_info :init) r)) r))
     (dolist (elem- _observation_infos)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get tug_observers_msgs::observer_info :md5sum-) "38c26f4d3dc2b7fc8f36eef35fb2083c")
(setf (get tug_observers_msgs::observer_info :datatype-) "tug_observers_msgs/observer_info")
(setf (get tug_observers_msgs::observer_info :definition-)
      "observation_info[] observation_infos

================================================================================
MSG: tug_observers_msgs/observation_info
Header header
string type
string resource
observation[] observation

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
MSG: tug_observers_msgs/observation
string observation_msg
string verbose_observation_msg
int32 observation
int32 GENERAL_OK=0
int32 GENERAL_ERROR=-1
int32 NO_STATE_FITS=-2
int32 NOT_AVAILABLE=-3
int32 TIMEOUT=-4

")



(provide :tug_observers_msgs/observer_info "38c26f4d3dc2b7fc8f36eef35fb2083c")


