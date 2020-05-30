
(cl:in-package :asdf)

(defsystem "tug_resource_monitor-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "NodesInfo" :depends-on ("_package_NodesInfo"))
    (:file "_package_NodesInfo" :depends-on ("_package"))
  ))