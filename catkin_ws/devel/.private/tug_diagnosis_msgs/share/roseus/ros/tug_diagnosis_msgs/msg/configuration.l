;; Auto-generated. Do not edit!


(when (boundp 'tug_diagnosis_msgs::configuration)
  (if (not (find-package "TUG_DIAGNOSIS_MSGS"))
    (make-package "TUG_DIAGNOSIS_MSGS"))
  (shadow 'configuration (find-package "TUG_DIAGNOSIS_MSGS")))
(unless (find-package "TUG_DIAGNOSIS_MSGS::CONFIGURATION")
  (make-package "TUG_DIAGNOSIS_MSGS::CONFIGURATION"))

(in-package "ROS")
;;//! \htmlinclude configuration.msg.html


(defclass tug_diagnosis_msgs::configuration
  :super ros::object
  :slots (_nodes _observers ))

(defmethod tug_diagnosis_msgs::configuration
  (:init
   (&key
    ((:nodes __nodes) (let (r) (dotimes (i 0) (push (instance tug_diagnosis_msgs::node_configuration :init) r)) r))
    ((:observers __observers) (let (r) (dotimes (i 0) (push (instance tug_diagnosis_msgs::observer_configuration :init) r)) r))
    )
   (send-super :init)
   (setq _nodes __nodes)
   (setq _observers __observers)
   self)
  (:nodes
   (&rest __nodes)
   (if (keywordp (car __nodes))
       (send* _nodes __nodes)
     (progn
       (if __nodes (setq _nodes (car __nodes)))
       _nodes)))
  (:observers
   (&rest __observers)
   (if (keywordp (car __observers))
       (send* _observers __observers)
     (progn
       (if __observers (setq _observers (car __observers)))
       _observers)))
  (:serialization-length
   ()
   (+
    ;; tug_diagnosis_msgs/node_configuration[] _nodes
    (apply #'+ (send-all _nodes :serialization-length)) 4
    ;; tug_diagnosis_msgs/observer_configuration[] _observers
    (apply #'+ (send-all _observers :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; tug_diagnosis_msgs/node_configuration[] _nodes
     (write-long (length _nodes) s)
     (dolist (elem _nodes)
       (send elem :serialize s)
       )
     ;; tug_diagnosis_msgs/observer_configuration[] _observers
     (write-long (length _observers) s)
     (dolist (elem _observers)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; tug_diagnosis_msgs/node_configuration[] _nodes
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _nodes (let (r) (dotimes (i n) (push (instance tug_diagnosis_msgs::node_configuration :init) r)) r))
     (dolist (elem- _nodes)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; tug_diagnosis_msgs/observer_configuration[] _observers
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _observers (let (r) (dotimes (i n) (push (instance tug_diagnosis_msgs::observer_configuration :init) r)) r))
     (dolist (elem- _observers)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get tug_diagnosis_msgs::configuration :md5sum-) "cadde33865d5cf9cef30c348d588d747")
(setf (get tug_diagnosis_msgs::configuration :datatype-) "tug_diagnosis_msgs/configuration")
(setf (get tug_diagnosis_msgs::configuration :definition-)
      "node_configuration[] nodes
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

")



(provide :tug_diagnosis_msgs/configuration "cadde33865d5cf9cef30c348d588d747")


