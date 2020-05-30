;; Auto-generated. Do not edit!


(when (boundp 'tug_diagnosis_msgs::observer_configuration)
  (if (not (find-package "TUG_DIAGNOSIS_MSGS"))
    (make-package "TUG_DIAGNOSIS_MSGS"))
  (shadow 'observer_configuration (find-package "TUG_DIAGNOSIS_MSGS")))
(unless (find-package "TUG_DIAGNOSIS_MSGS::OBSERVER_CONFIGURATION")
  (make-package "TUG_DIAGNOSIS_MSGS::OBSERVER_CONFIGURATION"))

(in-package "ROS")
;;//! \htmlinclude observer_configuration.msg.html


(defclass tug_diagnosis_msgs::observer_configuration
  :super ros::object
  :slots (_type _resource ))

(defmethod tug_diagnosis_msgs::observer_configuration
  (:init
   (&key
    ((:type __type) "")
    ((:resource __resource) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _type (string __type))
   (setq _resource __resource)
   self)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:resource
   (&optional __resource)
   (if __resource (setq _resource __resource)) _resource)
  (:serialization-length
   ()
   (+
    ;; string _type
    4 (length _type)
    ;; string[] _resource
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _resource)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _type
       (write-long (length _type) s) (princ _type s)
     ;; string[] _resource
     (write-long (length _resource) s)
     (dolist (elem _resource)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string[] _resource
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _resource (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _resource i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(setf (get tug_diagnosis_msgs::observer_configuration :md5sum-) "a2f5ebb423c16ad934f5d2cae333e2df")
(setf (get tug_diagnosis_msgs::observer_configuration :datatype-) "tug_diagnosis_msgs/observer_configuration")
(setf (get tug_diagnosis_msgs::observer_configuration :definition-)
      "string type
string[] resource

")



(provide :tug_diagnosis_msgs/observer_configuration "a2f5ebb423c16ad934f5d2cae333e2df")


