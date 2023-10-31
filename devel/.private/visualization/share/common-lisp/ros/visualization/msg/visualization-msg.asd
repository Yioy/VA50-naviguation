
(cl:in-package :asdf)

(defsystem "visualization-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "VizUpdate" :depends-on ("_package_VizUpdate"))
    (:file "_package_VizUpdate" :depends-on ("_package"))
  ))