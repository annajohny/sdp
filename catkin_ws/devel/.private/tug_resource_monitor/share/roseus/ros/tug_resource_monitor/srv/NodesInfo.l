;; Auto-generated. Do not edit!


(when (boundp 'tug_resource_monitor::NodesInfo)
  (if (not (find-package "TUG_RESOURCE_MONITOR"))
    (make-package "TUG_RESOURCE_MONITOR"))
  (shadow 'NodesInfo (find-package "TUG_RESOURCE_MONITOR")))
(unless (find-package "TUG_RESOURCE_MONITOR::NODESINFO")
  (make-package "TUG_RESOURCE_MONITOR::NODESINFO"))
(unless (find-package "TUG_RESOURCE_MONITOR::NODESINFOREQUEST")
  (make-package "TUG_RESOURCE_MONITOR::NODESINFOREQUEST"))
(unless (find-package "TUG_RESOURCE_MONITOR::NODESINFORESPONSE")
  (make-package "TUG_RESOURCE_MONITOR::NODESINFORESPONSE"))

(in-package "ROS")





(defclass tug_resource_monitor::NodesInfoRequest
  :super ros::object
  :slots (_node_names ))

(defmethod tug_resource_monitor::NodesInfoRequest
  (:init
   (&key
    ((:node_names __node_names) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _node_names __node_names)
   self)
  (:node_names
   (&optional __node_names)
   (if __node_names (setq _node_names __node_names)) _node_names)
  (:serialization-length
   ()
   (+
    ;; string[] _node_names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _node_names)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _node_names
     (write-long (length _node_names) s)
     (dolist (elem _node_names)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _node_names
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _node_names (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _node_names i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(defclass tug_resource_monitor::NodesInfoResponse
  :super ros::object
  :slots ())

(defmethod tug_resource_monitor::NodesInfoResponse
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

(defclass tug_resource_monitor::NodesInfo
  :super ros::object
  :slots ())

(setf (get tug_resource_monitor::NodesInfo :md5sum-) "42c13fd17030d24481c0409739ac5ae7")
(setf (get tug_resource_monitor::NodesInfo :datatype-) "tug_resource_monitor/NodesInfo")
(setf (get tug_resource_monitor::NodesInfo :request) tug_resource_monitor::NodesInfoRequest)
(setf (get tug_resource_monitor::NodesInfo :response) tug_resource_monitor::NodesInfoResponse)

(defmethod tug_resource_monitor::NodesInfoRequest
  (:response () (instance tug_resource_monitor::NodesInfoResponse :init)))

(setf (get tug_resource_monitor::NodesInfoRequest :md5sum-) "42c13fd17030d24481c0409739ac5ae7")
(setf (get tug_resource_monitor::NodesInfoRequest :datatype-) "tug_resource_monitor/NodesInfoRequest")
(setf (get tug_resource_monitor::NodesInfoRequest :definition-)
      "string[] node_names
---

")

(setf (get tug_resource_monitor::NodesInfoResponse :md5sum-) "42c13fd17030d24481c0409739ac5ae7")
(setf (get tug_resource_monitor::NodesInfoResponse :datatype-) "tug_resource_monitor/NodesInfoResponse")
(setf (get tug_resource_monitor::NodesInfoResponse :definition-)
      "string[] node_names
---

")



(provide :tug_resource_monitor/NodesInfo "42c13fd17030d24481c0409739ac5ae7")


