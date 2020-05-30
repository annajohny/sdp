;; Auto-generated. Do not edit!


(when (boundp 'tug_resource_monitor::NodeInfo)
  (if (not (find-package "TUG_RESOURCE_MONITOR"))
    (make-package "TUG_RESOURCE_MONITOR"))
  (shadow 'NodeInfo (find-package "TUG_RESOURCE_MONITOR")))
(unless (find-package "TUG_RESOURCE_MONITOR::NODEINFO")
  (make-package "TUG_RESOURCE_MONITOR::NODEINFO"))

(in-package "ROS")
;;//! \htmlinclude NodeInfo.msg.html


(intern "*NO_ERROR*" (find-package "TUG_RESOURCE_MONITOR::NODEINFO"))
(shadow '*NO_ERROR* (find-package "TUG_RESOURCE_MONITOR::NODEINFO"))
(defconstant tug_resource_monitor::NodeInfo::*NO_ERROR* 0)
(intern "*ERROR_PID_NOT_FOUND*" (find-package "TUG_RESOURCE_MONITOR::NODEINFO"))
(shadow '*ERROR_PID_NOT_FOUND* (find-package "TUG_RESOURCE_MONITOR::NODEINFO"))
(defconstant tug_resource_monitor::NodeInfo::*ERROR_PID_NOT_FOUND* 1)
(defclass tug_resource_monitor::NodeInfo
  :super ros::object
  :slots (_name _pid _hostname _cpu _memory _error ))

(defmethod tug_resource_monitor::NodeInfo
  (:init
   (&key
    ((:name __name) "")
    ((:pid __pid) 0)
    ((:hostname __hostname) "")
    ((:cpu __cpu) 0.0)
    ((:memory __memory) 0)
    ((:error __error) 0)
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _pid (round __pid))
   (setq _hostname (string __hostname))
   (setq _cpu (float __cpu))
   (setq _memory (round __memory))
   (setq _error (round __error))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:pid
   (&optional __pid)
   (if __pid (setq _pid __pid)) _pid)
  (:hostname
   (&optional __hostname)
   (if __hostname (setq _hostname __hostname)) _hostname)
  (:cpu
   (&optional __cpu)
   (if __cpu (setq _cpu __cpu)) _cpu)
  (:memory
   (&optional __memory)
   (if __memory (setq _memory __memory)) _memory)
  (:error
   (&optional __error)
   (if __error (setq _error __error)) _error)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; uint32 _pid
    4
    ;; string _hostname
    4 (length _hostname)
    ;; float32 _cpu
    4
    ;; uint64 _memory
    8
    ;; uint32 _error
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; uint32 _pid
       (write-long _pid s)
     ;; string _hostname
       (write-long (length _hostname) s) (princ _hostname s)
     ;; float32 _cpu
       (sys::poke _cpu (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint64 _memory
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _memory (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _memory) (= (length (_memory . bv)) 2)) ;; bignum
              (write-long (ash (elt (_memory . bv) 0) 0) s)
              (write-long (ash (elt (_memory . bv) 1) -1) s))
             ((and (class _memory) (= (length (_memory . bv)) 1)) ;; big1
              (write-long (elt (_memory . bv) 0) s)
              (write-long (if (>= _memory 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _memory s)(write-long (if (>= _memory 0) 0 #xffffffff) s)))
     ;; uint32 _error
       (write-long _error s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint32 _pid
     (setq _pid (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _hostname
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _hostname (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _cpu
     (setq _cpu (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint64 _memory
     
#+(or :alpha :irix6 :x86_64)
      (setf _memory (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _memory (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; uint32 _error
     (setq _error (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get tug_resource_monitor::NodeInfo :md5sum-) "b3df41a0cc3ca1f8f984ebc9825c7a08")
(setf (get tug_resource_monitor::NodeInfo :datatype-) "tug_resource_monitor/NodeInfo")
(setf (get tug_resource_monitor::NodeInfo :definition-)
      "string name
uint32 pid
string hostname
float32 cpu
uint64 memory
uint32 error
uint32 NO_ERROR=0
uint32 ERROR_PID_NOT_FOUND=1

")



(provide :tug_resource_monitor/NodeInfo "b3df41a0cc3ca1f8f984ebc9825c7a08")


