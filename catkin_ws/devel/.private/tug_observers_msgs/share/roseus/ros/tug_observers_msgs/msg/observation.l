;; Auto-generated. Do not edit!


(when (boundp 'tug_observers_msgs::observation)
  (if (not (find-package "TUG_OBSERVERS_MSGS"))
    (make-package "TUG_OBSERVERS_MSGS"))
  (shadow 'observation (find-package "TUG_OBSERVERS_MSGS")))
(unless (find-package "TUG_OBSERVERS_MSGS::OBSERVATION")
  (make-package "TUG_OBSERVERS_MSGS::OBSERVATION"))

(in-package "ROS")
;;//! \htmlinclude observation.msg.html


(intern "*GENERAL_OK*" (find-package "TUG_OBSERVERS_MSGS::OBSERVATION"))
(shadow '*GENERAL_OK* (find-package "TUG_OBSERVERS_MSGS::OBSERVATION"))
(defconstant tug_observers_msgs::observation::*GENERAL_OK* 0)
(intern "*GENERAL_ERROR*" (find-package "TUG_OBSERVERS_MSGS::OBSERVATION"))
(shadow '*GENERAL_ERROR* (find-package "TUG_OBSERVERS_MSGS::OBSERVATION"))
(defconstant tug_observers_msgs::observation::*GENERAL_ERROR* -1)
(intern "*NO_STATE_FITS*" (find-package "TUG_OBSERVERS_MSGS::OBSERVATION"))
(shadow '*NO_STATE_FITS* (find-package "TUG_OBSERVERS_MSGS::OBSERVATION"))
(defconstant tug_observers_msgs::observation::*NO_STATE_FITS* -2)
(intern "*NOT_AVAILABLE*" (find-package "TUG_OBSERVERS_MSGS::OBSERVATION"))
(shadow '*NOT_AVAILABLE* (find-package "TUG_OBSERVERS_MSGS::OBSERVATION"))
(defconstant tug_observers_msgs::observation::*NOT_AVAILABLE* -3)
(intern "*TIMEOUT*" (find-package "TUG_OBSERVERS_MSGS::OBSERVATION"))
(shadow '*TIMEOUT* (find-package "TUG_OBSERVERS_MSGS::OBSERVATION"))
(defconstant tug_observers_msgs::observation::*TIMEOUT* -4)
(defclass tug_observers_msgs::observation
  :super ros::object
  :slots (_observation_msg _verbose_observation_msg _observation ))

(defmethod tug_observers_msgs::observation
  (:init
   (&key
    ((:observation_msg __observation_msg) "")
    ((:verbose_observation_msg __verbose_observation_msg) "")
    ((:observation __observation) 0)
    )
   (send-super :init)
   (setq _observation_msg (string __observation_msg))
   (setq _verbose_observation_msg (string __verbose_observation_msg))
   (setq _observation (round __observation))
   self)
  (:observation_msg
   (&optional __observation_msg)
   (if __observation_msg (setq _observation_msg __observation_msg)) _observation_msg)
  (:verbose_observation_msg
   (&optional __verbose_observation_msg)
   (if __verbose_observation_msg (setq _verbose_observation_msg __verbose_observation_msg)) _verbose_observation_msg)
  (:observation
   (&optional __observation)
   (if __observation (setq _observation __observation)) _observation)
  (:serialization-length
   ()
   (+
    ;; string _observation_msg
    4 (length _observation_msg)
    ;; string _verbose_observation_msg
    4 (length _verbose_observation_msg)
    ;; int32 _observation
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _observation_msg
       (write-long (length _observation_msg) s) (princ _observation_msg s)
     ;; string _verbose_observation_msg
       (write-long (length _verbose_observation_msg) s) (princ _verbose_observation_msg s)
     ;; int32 _observation
       (write-long _observation s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _observation_msg
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _observation_msg (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _verbose_observation_msg
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _verbose_observation_msg (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _observation
     (setq _observation (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get tug_observers_msgs::observation :md5sum-) "176310c5b8642d2cf705c70f2da7bc39")
(setf (get tug_observers_msgs::observation :datatype-) "tug_observers_msgs/observation")
(setf (get tug_observers_msgs::observation :definition-)
      "string observation_msg
string verbose_observation_msg
int32 observation
int32 GENERAL_OK=0
int32 GENERAL_ERROR=-1
int32 NO_STATE_FITS=-2
int32 NOT_AVAILABLE=-3
int32 TIMEOUT=-4

")



(provide :tug_observers_msgs/observation "176310c5b8642d2cf705c70f2da7bc39")


