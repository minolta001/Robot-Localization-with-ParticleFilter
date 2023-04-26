
(cl:in-package :asdf)

(defsystem "motion_model-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "motion_model_msgs" :depends-on ("_package_motion_model_msgs"))
    (:file "_package_motion_model_msgs" :depends-on ("_package"))
  ))