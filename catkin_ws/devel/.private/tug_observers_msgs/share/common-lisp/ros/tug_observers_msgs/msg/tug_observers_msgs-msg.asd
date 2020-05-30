
(cl:in-package :asdf)

(defsystem "tug_observers_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "observation" :depends-on ("_package_observation"))
    (:file "_package_observation" :depends-on ("_package"))
    (:file "observation_info" :depends-on ("_package_observation_info"))
    (:file "_package_observation_info" :depends-on ("_package"))
    (:file "observer_info" :depends-on ("_package_observer_info"))
    (:file "_package_observer_info" :depends-on ("_package"))
  ))