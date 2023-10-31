
(cl:in-package :asdf)

(defsystem "circulation-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "TimeBatch" :depends-on ("_package_TimeBatch"))
    (:file "_package_TimeBatch" :depends-on ("_package"))
    (:file "Trajectory" :depends-on ("_package_Trajectory"))
    (:file "_package_Trajectory" :depends-on ("_package"))
  ))