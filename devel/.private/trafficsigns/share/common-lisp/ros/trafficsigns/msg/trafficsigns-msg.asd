
(cl:in-package :asdf)

(defsystem "trafficsigns-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "TrafficSign" :depends-on ("_package_TrafficSign"))
    (:file "_package_TrafficSign" :depends-on ("_package"))
    (:file "TrafficSignStatus" :depends-on ("_package_TrafficSignStatus"))
    (:file "_package_TrafficSignStatus" :depends-on ("_package"))
  ))