
(cl:in-package :asdf)

(defsystem "tug_diagnosis_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :tug_diagnosis_msgs-msg
)
  :components ((:file "_package")
    (:file "DiagnosisConfiguration" :depends-on ("_package_DiagnosisConfiguration"))
    (:file "_package_DiagnosisConfiguration" :depends-on ("_package"))
  ))