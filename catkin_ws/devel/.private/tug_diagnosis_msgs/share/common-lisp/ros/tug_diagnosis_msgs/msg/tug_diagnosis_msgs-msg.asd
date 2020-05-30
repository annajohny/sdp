
(cl:in-package :asdf)

(defsystem "tug_diagnosis_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "configuration" :depends-on ("_package_configuration"))
    (:file "_package_configuration" :depends-on ("_package"))
    (:file "diagnosis" :depends-on ("_package_diagnosis"))
    (:file "_package_diagnosis" :depends-on ("_package"))
    (:file "diagnosis_set" :depends-on ("_package_diagnosis_set"))
    (:file "_package_diagnosis_set" :depends-on ("_package"))
    (:file "node_configuration" :depends-on ("_package_node_configuration"))
    (:file "_package_node_configuration" :depends-on ("_package"))
    (:file "observer_configuration" :depends-on ("_package_observer_configuration"))
    (:file "_package_observer_configuration" :depends-on ("_package"))
    (:file "resource_mode_assignement" :depends-on ("_package_resource_mode_assignement"))
    (:file "_package_resource_mode_assignement" :depends-on ("_package"))
  ))