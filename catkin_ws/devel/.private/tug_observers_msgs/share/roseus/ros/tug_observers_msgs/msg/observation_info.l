;; Auto-generated. Do not edit!


(when (boundp 'tug_observers_msgs::observation_info)
  (if (not (find-package "TUG_OBSERVERS_MSGS"))
    (make-package "TUG_OBSERVERS_MSGS"))
  (shadow 'observation_info (find-package "TUG_OBSERVERS_MSGS")))
(unless (find-package "TUG_OBSERVERS_MSGS::OBSERVATION_INFO")
  (make-package "TUG_OBSERVERS_MSGS::OBSERVATION_INFO"))

(in-package "ROS")
;;//! \htmlinclude observation_info.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass tug_observers_msgs::observation_info
  :super ros::object
  :slots (_header _type _resource _observation ))

(defmethod tug_observers_msgs::observation_info
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:type __type) "")
    ((:resource __resource) "")
    ((:observation __observation) (let (r) (dotimes (i 0) (push (instance tug_observers_msgs::observation :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _type (string __type))
   (setq _resource (string __resource))
   (setq _observation __observation)
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
  (:resource
   (&optional __resource)
   (if __resource (setq _resource __resource)) _resource)
  (:observation
   (&rest __observation)
   (if (keywordp (car __observation))
       (send* _observation __observation)
     (progn
       (if __observation (setq _observation (car __observation)))
       _observation)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _type
    4 (length _type)
    ;; string _resource
    4 (length _resource)
    ;; tug_observers_msgs/observation[] _observation
    (apply #'+ (send-all _observation :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _type
       (write-long (length _type) s) (princ _type s)
     ;; string _resource
       (write-long (length _resource) s) (princ _resource s)
     ;; tug_observers_msgs/observation[] _observation
     (write-long (length _observation) s)
     (dolist (elem _observation)
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
   ;; string _resource
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _resource (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; tug_observers_msgs/observation[] _observation
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _observation (let (r) (dotimes (i n) (push (instance tug_observers_msgs::observation :init) r)) r))
     (dolist (elem- _observation)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get tug_observers_msgs::observation_info :md5sum-) "920b39a3d493095fc494ca757b21762f")
(setf (get tug_observers_msgs::observation_info :datatype-) "tug_observers_msgs/observation_info")
(setf (get tug_observers_msgs::observation_info :definition-)
      "Header header
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



(provide :tug_observers_msgs/observation_info "920b39a3d493095fc494ca757b21762f")


