;; Auto-generated. Do not edit!


(when (boundp 'tug_diagnosis_msgs::diagnosis)
  (if (not (find-package "TUG_DIAGNOSIS_MSGS"))
    (make-package "TUG_DIAGNOSIS_MSGS"))
  (shadow 'diagnosis (find-package "TUG_DIAGNOSIS_MSGS")))
(unless (find-package "TUG_DIAGNOSIS_MSGS::DIAGNOSIS")
  (make-package "TUG_DIAGNOSIS_MSGS::DIAGNOSIS"))

(in-package "ROS")
;;//! \htmlinclude diagnosis.msg.html


(defclass tug_diagnosis_msgs::diagnosis
  :super ros::object
  :slots (_diagnosis ))

(defmethod tug_diagnosis_msgs::diagnosis
  (:init
   (&key
    ((:diagnosis __diagnosis) (let (r) (dotimes (i 0) (push (instance tug_diagnosis_msgs::resource_mode_assignement :init) r)) r))
    )
   (send-super :init)
   (setq _diagnosis __diagnosis)
   self)
  (:diagnosis
   (&rest __diagnosis)
   (if (keywordp (car __diagnosis))
       (send* _diagnosis __diagnosis)
     (progn
       (if __diagnosis (setq _diagnosis (car __diagnosis)))
       _diagnosis)))
  (:serialization-length
   ()
   (+
    ;; tug_diagnosis_msgs/resource_mode_assignement[] _diagnosis
    (apply #'+ (send-all _diagnosis :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; tug_diagnosis_msgs/resource_mode_assignement[] _diagnosis
     (write-long (length _diagnosis) s)
     (dolist (elem _diagnosis)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; tug_diagnosis_msgs/resource_mode_assignement[] _diagnosis
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _diagnosis (let (r) (dotimes (i n) (push (instance tug_diagnosis_msgs::resource_mode_assignement :init) r)) r))
     (dolist (elem- _diagnosis)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get tug_diagnosis_msgs::diagnosis :md5sum-) "52161f8a0d1740c6fc459d4f815e3e6d")
(setf (get tug_diagnosis_msgs::diagnosis :datatype-) "tug_diagnosis_msgs/diagnosis")
(setf (get tug_diagnosis_msgs::diagnosis :definition-)
      "resource_mode_assignement[] diagnosis

================================================================================
MSG: tug_diagnosis_msgs/resource_mode_assignement
string resource
string mode_msg
int32 mode
int32 GENERAL_OK=0
int32 GENERAL_ERROR=-1

")



(provide :tug_diagnosis_msgs/diagnosis "52161f8a0d1740c6fc459d4f815e3e6d")


