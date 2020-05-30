;; Auto-generated. Do not edit!


(when (boundp 'tug_diagnosis_msgs::resource_mode_assignement)
  (if (not (find-package "TUG_DIAGNOSIS_MSGS"))
    (make-package "TUG_DIAGNOSIS_MSGS"))
  (shadow 'resource_mode_assignement (find-package "TUG_DIAGNOSIS_MSGS")))
(unless (find-package "TUG_DIAGNOSIS_MSGS::RESOURCE_MODE_ASSIGNEMENT")
  (make-package "TUG_DIAGNOSIS_MSGS::RESOURCE_MODE_ASSIGNEMENT"))

(in-package "ROS")
;;//! \htmlinclude resource_mode_assignement.msg.html


(intern "*GENERAL_OK*" (find-package "TUG_DIAGNOSIS_MSGS::RESOURCE_MODE_ASSIGNEMENT"))
(shadow '*GENERAL_OK* (find-package "TUG_DIAGNOSIS_MSGS::RESOURCE_MODE_ASSIGNEMENT"))
(defconstant tug_diagnosis_msgs::resource_mode_assignement::*GENERAL_OK* 0)
(intern "*GENERAL_ERROR*" (find-package "TUG_DIAGNOSIS_MSGS::RESOURCE_MODE_ASSIGNEMENT"))
(shadow '*GENERAL_ERROR* (find-package "TUG_DIAGNOSIS_MSGS::RESOURCE_MODE_ASSIGNEMENT"))
(defconstant tug_diagnosis_msgs::resource_mode_assignement::*GENERAL_ERROR* -1)
(defclass tug_diagnosis_msgs::resource_mode_assignement
  :super ros::object
  :slots (_resource _mode_msg _mode ))

(defmethod tug_diagnosis_msgs::resource_mode_assignement
  (:init
   (&key
    ((:resource __resource) "")
    ((:mode_msg __mode_msg) "")
    ((:mode __mode) 0)
    )
   (send-super :init)
   (setq _resource (string __resource))
   (setq _mode_msg (string __mode_msg))
   (setq _mode (round __mode))
   self)
  (:resource
   (&optional __resource)
   (if __resource (setq _resource __resource)) _resource)
  (:mode_msg
   (&optional __mode_msg)
   (if __mode_msg (setq _mode_msg __mode_msg)) _mode_msg)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:serialization-length
   ()
   (+
    ;; string _resource
    4 (length _resource)
    ;; string _mode_msg
    4 (length _mode_msg)
    ;; int32 _mode
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _resource
       (write-long (length _resource) s) (princ _resource s)
     ;; string _mode_msg
       (write-long (length _mode_msg) s) (princ _mode_msg s)
     ;; int32 _mode
       (write-long _mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _resource
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _resource (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _mode_msg
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _mode_msg (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _mode
     (setq _mode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get tug_diagnosis_msgs::resource_mode_assignement :md5sum-) "3bb0fd0cf9c222fef578a66ea18f38c0")
(setf (get tug_diagnosis_msgs::resource_mode_assignement :datatype-) "tug_diagnosis_msgs/resource_mode_assignement")
(setf (get tug_diagnosis_msgs::resource_mode_assignement :definition-)
      "string resource
string mode_msg
int32 mode
int32 GENERAL_OK=0
int32 GENERAL_ERROR=-1

")



(provide :tug_diagnosis_msgs/resource_mode_assignement "3bb0fd0cf9c222fef578a66ea18f38c0")


