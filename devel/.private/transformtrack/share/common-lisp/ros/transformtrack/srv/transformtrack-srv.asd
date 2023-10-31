
(cl:in-package :asdf)

(defsystem "transformtrack-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "DropVelocity" :depends-on ("_package_DropVelocity"))
    (:file "_package_DropVelocity" :depends-on ("_package"))
    (:file "TransformBatch" :depends-on ("_package_TransformBatch"))
    (:file "_package_TransformBatch" :depends-on ("_package"))
  ))