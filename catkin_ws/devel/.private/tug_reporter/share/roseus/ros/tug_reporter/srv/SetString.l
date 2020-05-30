;; Auto-generated. Do not edit!


(when (boundp 'tug_reporter::SetString)
  (if (not (find-package "TUG_REPORTER"))
    (make-package "TUG_REPORTER"))
  (shadow 'SetString (find-package "TUG_REPORTER")))
(unless (find-package "TUG_REPORTER::SETSTRING")
  (make-package "TUG_REPORTER::SETSTRING"))
(unless (find-package "TUG_REPORTER::SETSTRINGREQUEST")
  (make-package "TUG_REPORTER::SETSTRINGREQUEST"))
(unless (find-package "TUG_REPORTER::SETSTRINGRESPONSE")
  (make-package "TUG_REPORTER::SETSTRINGRESPONSE"))

(in-package "ROS")





(defclass tug_reporter::SetStringRequest
  :super ros::object
  :slots (_message ))

(defmethod tug_reporter::SetStringRequest
  (:init
   (&key
    ((:message __message) "")
    )
   (send-super :init)
   (setq _message (string __message))
   self)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass tug_reporter::SetStringResponse
  :super ros::object
  :slots ())

(defmethod tug_reporter::SetStringResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass tug_reporter::SetString
  :super ros::object
  :slots ())

(setf (get tug_reporter::SetString :md5sum-) "5f003d6bcc824cbd51361d66d8e4f76c")
(setf (get tug_reporter::SetString :datatype-) "tug_reporter/SetString")
(setf (get tug_reporter::SetString :request) tug_reporter::SetStringRequest)
(setf (get tug_reporter::SetString :response) tug_reporter::SetStringResponse)

(defmethod tug_reporter::SetStringRequest
  (:response () (instance tug_reporter::SetStringResponse :init)))

(setf (get tug_reporter::SetStringRequest :md5sum-) "5f003d6bcc824cbd51361d66d8e4f76c")
(setf (get tug_reporter::SetStringRequest :datatype-) "tug_reporter/SetStringRequest")
(setf (get tug_reporter::SetStringRequest :definition-)
      "string message
---
")

(setf (get tug_reporter::SetStringResponse :md5sum-) "5f003d6bcc824cbd51361d66d8e4f76c")
(setf (get tug_reporter::SetStringResponse :datatype-) "tug_reporter/SetStringResponse")
(setf (get tug_reporter::SetStringResponse :definition-)
      "string message
---
")



(provide :tug_reporter/SetString "5f003d6bcc824cbd51361d66d8e4f76c")


