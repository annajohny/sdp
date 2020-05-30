
(cl:in-package :asdf)

(defsystem "tug_resource_monitor-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "NodeInfo" :depends-on ("_package_NodeInfo"))
    (:file "_package_NodeInfo" :depends-on ("_package"))
    (:file "NodeInfoArray" :depends-on ("_package_NodeInfoArray"))
    (:file "_package_NodeInfoArray" :depends-on ("_package"))
  ))