;; Auto-generated. Do not edit!


(when (boundp 'trafficsigns::TrafficSignStatus)
  (if (not (find-package "TRAFFICSIGNS"))
    (make-package "TRAFFICSIGNS"))
  (shadow 'TrafficSignStatus (find-package "TRAFFICSIGNS")))
(unless (find-package "TRAFFICSIGNS::TRAFFICSIGNSTATUS")
  (make-package "TRAFFICSIGNS::TRAFFICSIGNSTATUS"))

(in-package "ROS")
;;//! \htmlinclude TrafficSignStatus.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass trafficsigns::TrafficSignStatus
  :super ros::object
  :slots (_header _traffic_signs ))

(defmethod trafficsigns::TrafficSignStatus
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:traffic_signs __traffic_signs) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _traffic_signs __traffic_signs)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:traffic_signs
   (&rest __traffic_signs)
   (if (keywordp (car __traffic_signs))
       (send* _traffic_signs __traffic_signs)
     (progn
       (if __traffic_signs (setq _traffic_signs (car __traffic_signs)))
       _traffic_signs)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; trafficsigns/TrafficSign[] _traffic_signs
    (apply #'+ (send-all _traffic_signs :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; trafficsigns/TrafficSign[] _traffic_signs
     (write-long (length _traffic_signs) s)
     (dolist (elem _traffic_signs)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; trafficsigns/TrafficSign[] _traffic_signs
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _traffic_signs (let (r) (dotimes (i n) (push (instance trafficsigns::TrafficSign :init) r)) r))
     (dolist (elem- _traffic_signs)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get trafficsigns::TrafficSignStatus :md5sum-) "481cb23ff8b604083652da43030d8361")
(setf (get trafficsigns::TrafficSignStatus :datatype-) "trafficsigns/TrafficSignStatus")
(setf (get trafficsigns::TrafficSignStatus :definition-)
      "std_msgs/Header header
TrafficSign[] traffic_signs
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: trafficsigns/TrafficSign
string category
string type
float32 x
float32 y
float32 z
float32 confidence
")



(provide :trafficsigns/TrafficSignStatus "481cb23ff8b604083652da43030d8361")


