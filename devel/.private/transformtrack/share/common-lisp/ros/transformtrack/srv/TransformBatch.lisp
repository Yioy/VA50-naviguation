; Auto-generated. Do not edit!


(cl:in-package transformtrack-srv)


;//! \htmlinclude TransformBatch-request.msg.html

(cl:defclass <TransformBatch-request> (roslisp-msg-protocol:ros-message)
  ((start_times
    :reader start_times
    :initarg :start_times
    :type (cl:vector cl:real)
   :initform (cl:make-array 0 :element-type 'cl:real :initial-element 0))
   (end_time
    :reader end_time
    :initarg :end_time
    :type cl:real
    :initform 0))
)

(cl:defclass TransformBatch-request (<TransformBatch-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TransformBatch-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TransformBatch-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name transformtrack-srv:<TransformBatch-request> is deprecated: use transformtrack-srv:TransformBatch-request instead.")))

(cl:ensure-generic-function 'start_times-val :lambda-list '(m))
(cl:defmethod start_times-val ((m <TransformBatch-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader transformtrack-srv:start_times-val is deprecated.  Use transformtrack-srv:start_times instead.")
  (start_times m))

(cl:ensure-generic-function 'end_time-val :lambda-list '(m))
(cl:defmethod end_time-val ((m <TransformBatch-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader transformtrack-srv:end_time-val is deprecated.  Use transformtrack-srv:end_time instead.")
  (end_time m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TransformBatch-request>) ostream)
  "Serializes a message object of type '<TransformBatch-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'start_times))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__sec (cl:floor ele))
        (__nsec (cl:round (cl:* 1e9 (cl:- ele (cl:floor ele))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream)))
   (cl:slot-value msg 'start_times))
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'end_time)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'end_time) (cl:floor (cl:slot-value msg 'end_time)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TransformBatch-request>) istream)
  "Deserializes a message object of type '<TransformBatch-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'start_times) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'start_times)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9)))))))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'end_time) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TransformBatch-request>)))
  "Returns string type for a service object of type '<TransformBatch-request>"
  "transformtrack/TransformBatchRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TransformBatch-request)))
  "Returns string type for a service object of type 'TransformBatch-request"
  "transformtrack/TransformBatchRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TransformBatch-request>)))
  "Returns md5sum for a message object of type '<TransformBatch-request>"
  "d17c54b06f263044d75d6d03617a9ba1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TransformBatch-request)))
  "Returns md5sum for a message object of type 'TransformBatch-request"
  "d17c54b06f263044d75d6d03617a9ba1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TransformBatch-request>)))
  "Returns full string definition for message of type '<TransformBatch-request>"
  (cl:format cl:nil "time[] start_times~%time end_time~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TransformBatch-request)))
  "Returns full string definition for message of type 'TransformBatch-request"
  (cl:format cl:nil "time[] start_times~%time end_time~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TransformBatch-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'start_times) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TransformBatch-request>))
  "Converts a ROS message object to a list"
  (cl:list 'TransformBatch-request
    (cl:cons ':start_times (start_times msg))
    (cl:cons ':end_time (end_time msg))
))
;//! \htmlinclude TransformBatch-response.msg.html

(cl:defclass <TransformBatch-response> (roslisp-msg-protocol:ros-message)
  ((transforms
    :reader transforms
    :initarg :transforms
    :type std_msgs-msg:Float64MultiArray
    :initform (cl:make-instance 'std_msgs-msg:Float64MultiArray))
   (distances
    :reader distances
    :initarg :distances
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass TransformBatch-response (<TransformBatch-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TransformBatch-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TransformBatch-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name transformtrack-srv:<TransformBatch-response> is deprecated: use transformtrack-srv:TransformBatch-response instead.")))

(cl:ensure-generic-function 'transforms-val :lambda-list '(m))
(cl:defmethod transforms-val ((m <TransformBatch-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader transformtrack-srv:transforms-val is deprecated.  Use transformtrack-srv:transforms instead.")
  (transforms m))

(cl:ensure-generic-function 'distances-val :lambda-list '(m))
(cl:defmethod distances-val ((m <TransformBatch-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader transformtrack-srv:distances-val is deprecated.  Use transformtrack-srv:distances instead.")
  (distances m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TransformBatch-response>) ostream)
  "Serializes a message object of type '<TransformBatch-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'transforms) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'distances))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'distances))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TransformBatch-response>) istream)
  "Deserializes a message object of type '<TransformBatch-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'transforms) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'distances) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'distances)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TransformBatch-response>)))
  "Returns string type for a service object of type '<TransformBatch-response>"
  "transformtrack/TransformBatchResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TransformBatch-response)))
  "Returns string type for a service object of type 'TransformBatch-response"
  "transformtrack/TransformBatchResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TransformBatch-response>)))
  "Returns md5sum for a message object of type '<TransformBatch-response>"
  "d17c54b06f263044d75d6d03617a9ba1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TransformBatch-response)))
  "Returns md5sum for a message object of type 'TransformBatch-response"
  "d17c54b06f263044d75d6d03617a9ba1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TransformBatch-response>)))
  "Returns full string definition for message of type '<TransformBatch-response>"
  (cl:format cl:nil "std_msgs/Float64MultiArray transforms~%float64[] distances~%~%================================================================================~%MSG: std_msgs/Float64MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float64[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TransformBatch-response)))
  "Returns full string definition for message of type 'TransformBatch-response"
  (cl:format cl:nil "std_msgs/Float64MultiArray transforms~%float64[] distances~%~%================================================================================~%MSG: std_msgs/Float64MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float64[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TransformBatch-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'transforms))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'distances) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TransformBatch-response>))
  "Converts a ROS message object to a list"
  (cl:list 'TransformBatch-response
    (cl:cons ':transforms (transforms msg))
    (cl:cons ':distances (distances msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'TransformBatch)))
  'TransformBatch-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'TransformBatch)))
  'TransformBatch-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TransformBatch)))
  "Returns string type for a service object of type '<TransformBatch>"
  "transformtrack/TransformBatch")