;; Auto-generated. Do not edit!


(when (boundp 'transformtrack::DropVelocity)
  (if (not (find-package "TRANSFORMTRACK"))
    (make-package "TRANSFORMTRACK"))
  (shadow 'DropVelocity (find-package "TRANSFORMTRACK")))
(unless (find-package "TRANSFORMTRACK::DROPVELOCITY")
  (make-package "TRANSFORMTRACK::DROPVELOCITY"))
(unless (find-package "TRANSFORMTRACK::DROPVELOCITYREQUEST")
  (make-package "TRANSFORMTRACK::DROPVELOCITYREQUEST"))
(unless (find-package "TRANSFORMTRACK::DROPVELOCITYRESPONSE")
  (make-package "TRANSFORMTRACK::DROPVELOCITYRESPONSE"))

(in-package "ROS")





(defclass transformtrack::DropVelocityRequest
  :super ros::object
  :slots (_end_time _unbias ))

(defmethod transformtrack::DropVelocityRequest
  (:init
   (&key
    ((:end_time __end_time) (instance ros::time :init))
    ((:unbias __unbias) nil)
    )
   (send-super :init)
   (setq _end_time __end_time)
   (setq _unbias __unbias)
   self)
  (:end_time
   (&optional __end_time)
   (if __end_time (setq _end_time __end_time)) _end_time)
  (:unbias
   (&optional (__unbias :null))
   (if (not (eq __unbias :null)) (setq _unbias __unbias)) _unbias)
  (:serialization-length
   ()
   (+
    ;; time _end_time
    8
    ;; bool _unbias
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _end_time
       (write-long (send _end_time :sec) s) (write-long (send _end_time :nsec) s)
     ;; bool _unbias
       (if _unbias (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _end_time
     (send _end_time :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _end_time :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _unbias
     (setq _unbias (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass transformtrack::DropVelocityResponse
  :super ros::object
  :slots (_done ))

(defmethod transformtrack::DropVelocityResponse
  (:init
   (&key
    ((:done __done) nil)
    )
   (send-super :init)
   (setq _done __done)
   self)
  (:done
   (&optional (__done :null))
   (if (not (eq __done :null)) (setq _done __done)) _done)
  (:serialization-length
   ()
   (+
    ;; bool _done
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _done
       (if _done (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _done
     (setq _done (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass transformtrack::DropVelocity
  :super ros::object
  :slots ())

(setf (get transformtrack::DropVelocity :md5sum-) "2b47bafdc4c00e27a7f0cfd76dc9037e")
(setf (get transformtrack::DropVelocity :datatype-) "transformtrack/DropVelocity")
(setf (get transformtrack::DropVelocity :request) transformtrack::DropVelocityRequest)
(setf (get transformtrack::DropVelocity :response) transformtrack::DropVelocityResponse)

(defmethod transformtrack::DropVelocityRequest
  (:response () (instance transformtrack::DropVelocityResponse :init)))

(setf (get transformtrack::DropVelocityRequest :md5sum-) "2b47bafdc4c00e27a7f0cfd76dc9037e")
(setf (get transformtrack::DropVelocityRequest :datatype-) "transformtrack/DropVelocityRequest")
(setf (get transformtrack::DropVelocityRequest :definition-)
      "time end_time
bool unbias
---
bool done
")

(setf (get transformtrack::DropVelocityResponse :md5sum-) "2b47bafdc4c00e27a7f0cfd76dc9037e")
(setf (get transformtrack::DropVelocityResponse :datatype-) "transformtrack/DropVelocityResponse")
(setf (get transformtrack::DropVelocityResponse :definition-)
      "time end_time
bool unbias
---
bool done
")



(provide :transformtrack/DropVelocity "2b47bafdc4c00e27a7f0cfd76dc9037e")


