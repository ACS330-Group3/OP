
(cl:in-package :asdf)

(defsystem "opl_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "LocationStatusOp" :depends-on ("_package_LocationStatusOp"))
    (:file "_package_LocationStatusOp" :depends-on ("_package"))
  ))