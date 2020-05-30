;; Auto-generated. Do not edit!


(when (boundp 'tug_diagnosis_msgs::node_configuration)
  (if (not (find-package "TUG_DIAGNOSIS_MSGS"))
    (make-package "TUG_DIAGNOSIS_MSGS"))
  (shadow 'node_configuration (find-package "TUG_DIAGNOSIS_MSGS")))
(unless (find-package "TUG_DIAGNOSIS_MSGS::NODE_CONFIGURATION")
  (make-package "TUG_DIAGNOSIS_MSGS::NODE_CONFIGURATION"))

(in-package "ROS")
;;//! \htmlinclude node_configuration.msg.html


(defclass tug_diagnosis_msgs::node_configuration
  :super ros::object
  :slots (_name _sub_topic _pub_topic ))

(defmethod tug_diagnosis_msgs::node_configuration
  (:init
   (&key
    ((:name __name) "")
    ((:sub_topic __sub_topic) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:pub_topic __pub_topic) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _sub_topic __sub_topic)
   (setq _pub_topic __pub_topic)
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:sub_topic
   (&optional __sub_topic)
   (if __sub_topic (setq _sub_topic __sub_topic)) _sub_topic)
  (:pub_topic
   (&optional __pub_topic)
   (if __pub_topic (setq _pub_topic __pub_topic)) _pub_topic)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; string[] _sub_topic
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _sub_topic)) 4
    ;; string[] _pub_topic
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _pub_topic)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; string[] _sub_topic
     (write-long (length _sub_topic) s)
     (dolist (elem _sub_topic)
       (write-long (length elem) s) (princ elem s)
       )
     ;; string[] _pub_topic
     (write-long (length _pub_topic) s)
     (dolist (elem _pub_topic)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string[] _sub_topic
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _sub_topic (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _sub_topic i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; string[] _pub_topic
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _pub_topic (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _pub_topic i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(setf (get tug_diagnosis_msgs::node_configuration :md5sum-) "d437a072fcff0973a4326ed765667e76")
(setf (get tug_diagnosis_msgs::node_configuration :datatype-) "tug_diagnosis_msgs/node_configuration")
(setf (get tug_diagnosis_msgs::node_configuration :definition-)
      "string name
string[] sub_topic
string[] pub_topic

")



(provide :tug_diagnosis_msgs/node_configuration "d437a072fcff0973a4326ed765667e76")


