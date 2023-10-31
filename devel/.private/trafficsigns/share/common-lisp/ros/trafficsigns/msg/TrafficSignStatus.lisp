; Auto-generated. Do not edit!


(cl:in-package trafficsigns-msg)


;//! \htmlinclude TrafficSignStatus.msg.html

(cl:defclass <TrafficSignStatus> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (traffic_signs
    :reader traffic_signs
    :initarg :traffic_signs
    :type (cl:vector trafficsigns-msg:TrafficSign)
   :initform (cl:make-array 0 :element-type 'trafficsigns-msg:TrafficSign :initial-element (cl:make-instance 'trafficsigns-msg:TrafficSign))))
)

(cl:defclass TrafficSignStatus (<TrafficSignStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrafficSignStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrafficSignStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name trafficsigns-msg:<TrafficSignStatus> is deprecated: use trafficsigns-msg:TrafficSignStatus instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <TrafficSignStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader trafficsigns-msg:header-val is deprecated.  Use trafficsigns-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'traffic_signs-val :lambda-list '(m))
(cl:defmethod traffic_signs-val ((m <TrafficSignStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader trafficsigns-msg:traffic_signs-val is deprecated.  Use trafficsigns-msg:traffic_signs instead.")
  (traffic_signs m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrafficSignStatus>) ostream)
  "Serializes a message object of type '<TrafficSignStatus>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'traffic_signs))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'traffic_signs))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrafficSignStatus>) istream)
  "Deserializes a message object of type '<TrafficSignStatus>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'traffic_signs) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'traffic_signs)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'trafficsigns-msg:TrafficSign))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrafficSignStatus>)))
  "Returns string type for a message object of type '<TrafficSignStatus>"
  "trafficsigns/TrafficSignStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrafficSignStatus)))
  "Returns string type for a message object of type 'TrafficSignStatus"
  "trafficsigns/TrafficSignStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrafficSignStatus>)))
  "Returns md5sum for a message object of type '<TrafficSignStatus>"
  "481cb23ff8b604083652da43030d8361")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrafficSignStatus)))
  "Returns md5sum for a message object of type 'TrafficSignStatus"
  "481cb23ff8b604083652da43030d8361")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrafficSignStatus>)))
  "Returns full string definition for message of type '<TrafficSignStatus>"
  (cl:format cl:nil "std_msgs/Header header~%TrafficSign[] traffic_signs~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: trafficsigns/TrafficSign~%string category~%string type~%float32 x~%float32 y~%float32 z~%float32 confidence~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrafficSignStatus)))
  "Returns full string definition for message of type 'TrafficSignStatus"
  (cl:format cl:nil "std_msgs/Header header~%TrafficSign[] traffic_signs~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: trafficsigns/TrafficSign~%string category~%string type~%float32 x~%float32 y~%float32 z~%float32 confidence~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrafficSignStatus>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'traffic_signs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrafficSignStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'TrafficSignStatus
    (cl:cons ':header (header msg))
    (cl:cons ':traffic_signs (traffic_signs msg))
))
