;; Auto-generated. Do not edit!


(when (boundp 'tug_diagnosis_msgs::DiagnosisConfiguration)
  (if (not (find-package "TUG_DIAGNOSIS_MSGS"))
    (make-package "TUG_DIAGNOSIS_MSGS"))
  (shadow 'DiagnosisConfiguration (find-package "TUG_DIAGNOSIS_MSGS")))
(unless (find-package "TUG_DIAGNOSIS_MSGS::DIAGNOSISCONFIGURATION")
  (make-package "TUG_DIAGNOSIS_MSGS::DIAGNOSISCONFIGURATION"))
(unless (find-package "TUG_DIAGNOSIS_MSGS::DIAGNOSISCONFIGURATIONREQUEST")
  (make-package "TUG_DIAGNOSIS_MSGS::DIAGNOSISCONFIGURATIONREQUEST"))
(unless (find-package "TUG_DIAGNOSIS_MSGS::DIAGNOSISCONFIGURATIONRESPONSE")
  (make-package "TUG_DIAGNOSIS_MSGS::DIAGNOSISCONFIGURATIONRESPONSE"))

(in-package "ROS")





(intern "*ADD*" (find-package "TUG_DIAGNOSIS_MSGS::DIAGNOSISCONFIGURATIONREQUEST"))
(shadow '*ADD* (find-package "TUG_DIAGNOSIS_MSGS::DIAGNOSISCONFIGURATIONREQUEST"))
(defconstant tug_diagnosis_msgs::DiagnosisConfigurationRequest::*ADD* 1)
(intern "*REMOVE*" (find-package "TUG_DIAGNOSIS_MSGS::DIAGNOSISCONFIGURATIONREQUEST"))
(shadow '*REMOVE* (find-package "TUG_DIAGNOSIS_MSGS::DIAGNOSISCONFIGURATIONREQUEST"))
(defconstant tug_diagnosis_msgs::DiagnosisConfigurationRequest::*REMOVE* 2)
(intern "*SET*" (find-package "TUG_DIAGNOSIS_MSGS::DIAGNOSISCONFIGURATIONREQUEST"))
(shadow '*SET* (find-package "TUG_DIAGNOSIS_MSGS::DIAGNOSISCONFIGURATIONREQUEST"))
(defconstant tug_diagnosis_msgs::DiagnosisConfigurationRequest::*SET* 3)
(intern "*UPDATE*" (find-package "TUG_DIAGNOSIS_MSGS::DIAGNOSISCONFIGURATIONREQUEST"))
(shadow '*UPDATE* (find-package "TUG_DIAGNOSIS_MSGS::DIAGNOSISCONFIGURATIONREQUEST"))
(defconstant tug_diagnosis_msgs::DiagnosisConfigurationRequest::*UPDATE* 4)
(defclass tug_diagnosis_msgs::DiagnosisConfigurationRequest
  :super ros::object
  :slots (_config _action ))

(defmethod tug_diagnosis_msgs::DiagnosisConfigurationRequest
  (:init
   (&key
    ((:config __config) (instance tug_diagnosis_msgs::configuration :init))
    ((:action __action) 0)
    )
   (send-super :init)
   (setq _config __config)
   (setq _action (round __action))
   self)
  (:config
   (&rest __config)
   (if (keywordp (car __config))
       (send* _config __config)
     (progn
       (if __config (setq _config (car __config)))
       _config)))
  (:action
   (&optional __action)
   (if __action (setq _action __action)) _action)
  (:serialization-length
   ()
   (+
    ;; tug_diagnosis_msgs/configuration _config
    (send _config :serialization-length)
    ;; int32 _action
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; tug_diagnosis_msgs/configuration _config
       (send _config :serialize s)
     ;; int32 _action
       (write-long _action s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; tug_diagnosis_msgs/configuration _config
     (send _config :deserialize buf ptr-) (incf ptr- (send _config :serialization-length))
   ;; int32 _action
     (setq _action (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(intern "*NO_ERROR*" (find-package "TUG_DIAGNOSIS_MSGS::DIAGNOSISCONFIGURATIONRESPONSE"))
(shadow '*NO_ERROR* (find-package "TUG_DIAGNOSIS_MSGS::DIAGNOSISCONFIGURATIONRESPONSE"))
(defconstant tug_diagnosis_msgs::DiagnosisConfigurationResponse::*NO_ERROR* 0)
(intern "*GENERAL_ERROR*" (find-package "TUG_DIAGNOSIS_MSGS::DIAGNOSISCONFIGURATIONRESPONSE"))
(shadow '*GENERAL_ERROR* (find-package "TUG_DIAGNOSIS_MSGS::DIAGNOSISCONFIGURATIONRESPONSE"))
(defconstant tug_diagnosis_msgs::DiagnosisConfigurationResponse::*GENERAL_ERROR* -1)
(intern "*CONFIG_INVALID*" (find-package "TUG_DIAGNOSIS_MSGS::DIAGNOSISCONFIGURATIONRESPONSE"))
(shadow '*CONFIG_INVALID* (find-package "TUG_DIAGNOSIS_MSGS::DIAGNOSISCONFIGURATIONRESPONSE"))
(defconstant tug_diagnosis_msgs::DiagnosisConfigurationResponse::*CONFIG_INVALID* -2)
(defclass tug_diagnosis_msgs::DiagnosisConfigurationResponse
  :super ros::object
  :slots (_errorcode _error_msg ))

(defmethod tug_diagnosis_msgs::DiagnosisConfigurationResponse
  (:init
   (&key
    ((:errorcode __errorcode) 0)
    ((:error_msg __error_msg) "")
    )
   (send-super :init)
   (setq _errorcode (round __errorcode))
   (setq _error_msg (string __error_msg))
   self)
  (:errorcode
   (&optional __errorcode)
   (if __errorcode (setq _errorcode __errorcode)) _errorcode)
  (:error_msg
   (&optional __error_msg)
   (if __error_msg (setq _error_msg __error_msg)) _error_msg)
  (:serialization-length
   ()
   (+
    ;; int32 _errorcode
    4
    ;; string _error_msg
    4 (length _error_msg)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _errorcode
       (write-long _errorcode s)
     ;; string _error_msg
       (write-long (length _error_msg) s) (princ _error_msg s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _errorcode
     (setq _errorcode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _error_msg
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _error_msg (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass tug_diagnosis_msgs::DiagnosisConfiguration
  :super ros::object
  :slots ())

(setf (get tug_diagnosis_msgs::DiagnosisConfiguration :md5sum-) "43fad42cb6cdc7f9a57a24dd1daa5334")
(setf (get tug_diagnosis_msgs::DiagnosisConfiguration :datatype-) "tug_diagnosis_msgs/DiagnosisConfiguration")
(setf (get tug_diagnosis_msgs::DiagnosisConfiguration :request) tug_diagnosis_msgs::DiagnosisConfigurationRequest)
(setf (get tug_diagnosis_msgs::DiagnosisConfiguration :response) tug_diagnosis_msgs::DiagnosisConfigurationResponse)

(defmethod tug_diagnosis_msgs::DiagnosisConfigurationRequest
  (:response () (instance tug_diagnosis_msgs::DiagnosisConfigurationResponse :init)))

(setf (get tug_diagnosis_msgs::DiagnosisConfigurationRequest :md5sum-) "43fad42cb6cdc7f9a57a24dd1daa5334")
(setf (get tug_diagnosis_msgs::DiagnosisConfigurationRequest :datatype-) "tug_diagnosis_msgs/DiagnosisConfigurationRequest")
(setf (get tug_diagnosis_msgs::DiagnosisConfigurationRequest :definition-)
      "
configuration config


int32 action
int32 ADD=1
int32 REMOVE=2
int32 SET=3
int32 UPDATE=4


================================================================================
MSG: tug_diagnosis_msgs/configuration
node_configuration[] nodes
observer_configuration[] observers

================================================================================
MSG: tug_diagnosis_msgs/node_configuration
string name
string[] sub_topic
string[] pub_topic

================================================================================
MSG: tug_diagnosis_msgs/observer_configuration
string type
string[] resource
---


int32 errorcode
string error_msg
int32 NO_ERROR=0
int32 GENERAL_ERROR=-1
int32 CONFIG_INVALID=-2



")

(setf (get tug_diagnosis_msgs::DiagnosisConfigurationResponse :md5sum-) "43fad42cb6cdc7f9a57a24dd1daa5334")
(setf (get tug_diagnosis_msgs::DiagnosisConfigurationResponse :datatype-) "tug_diagnosis_msgs/DiagnosisConfigurationResponse")
(setf (get tug_diagnosis_msgs::DiagnosisConfigurationResponse :definition-)
      "
configuration config


int32 action
int32 ADD=1
int32 REMOVE=2
int32 SET=3
int32 UPDATE=4


================================================================================
MSG: tug_diagnosis_msgs/configuration
node_configuration[] nodes
observer_configuration[] observers

================================================================================
MSG: tug_diagnosis_msgs/node_configuration
string name
string[] sub_topic
string[] pub_topic

================================================================================
MSG: tug_diagnosis_msgs/observer_configuration
string type
string[] resource
---


int32 errorcode
string error_msg
int32 NO_ERROR=0
int32 GENERAL_ERROR=-1
int32 CONFIG_INVALID=-2



")



(provide :tug_diagnosis_msgs/DiagnosisConfiguration "43fad42cb6cdc7f9a57a24dd1daa5334")


