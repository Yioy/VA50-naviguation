; Auto-generated. Do not edit!


(cl:in-package transformtrack-srv)


;//! \htmlinclude DropVelocity-request.msg.html

(cl:defclass <DropVelocity-request> (roslisp-msg-protocol:ros-message)
  ((end_time
    :reader end_time
    :initarg :end_time
    :type cl:real
    :initform 0)
   (unbias
    :reader unbias
    :initarg :unbias
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass DropVelocity-request (<DropVelocity-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DropVelocity-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DropVelocity-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name transformtrack-srv:<DropVelocity-request> is deprecated: use transformtrack-srv:DropVelocity-request instead.")))

(cl:ensure-generic-function 'end_time-val :lambda-list '(m))
(cl:defmethod end_time-val ((m <DropVelocity-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader transformtrack-srv:end_time-val is deprecated.  Use transformtrack-srv:end_time instead.")
  (end_time m))

(cl:ensure-generic-function 'unbias-val :lambda-list '(m))
(cl:defmethod unbias-val ((m <DropVelocity-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader transformtrack-srv:unbias-val is deprecated.  Use transformtrack-srv:unbias instead.")
  (unbias m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DropVelocity-request>) ostream)
  "Serializes a message object of type '<DropVelocity-request>"
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
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'unbias) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DropVelocity-request>) istream)
  "Deserializes a message object of type '<DropVelocity-request>"
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
    (cl:setf (cl:slot-value msg 'unbias) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DropVelocity-request>)))
  "Returns string type for a service object of type '<DropVelocity-request>"
  "transformtrack/DropVelocityRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DropVelocity-request)))
  "Returns string type for a service object of type 'DropVelocity-request"
  "transformtrack/DropVelocityRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DropVelocity-request>)))
  "Returns md5sum for a message object of type '<DropVelocity-request>"
  "2b47bafdc4c00e27a7f0cfd76dc9037e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DropVelocity-request)))
  "Returns md5sum for a message object of type 'DropVelocity-request"
  "2b47bafdc4c00e27a7f0cfd76dc9037e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DropVelocity-request>)))
  "Returns full string definition for message of type '<DropVelocity-request>"
  (cl:format cl:nil "time end_time~%bool unbias~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DropVelocity-request)))
  "Returns full string definition for message of type 'DropVelocity-request"
  (cl:format cl:nil "time end_time~%bool unbias~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DropVelocity-request>))
  (cl:+ 0
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DropVelocity-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DropVelocity-request
    (cl:cons ':end_time (end_time msg))
    (cl:cons ':unbias (unbias msg))
))
;//! \htmlinclude DropVelocity-response.msg.html

(cl:defclass <DropVelocity-response> (roslisp-msg-protocol:ros-message)
  ((done
    :reader done
    :initarg :done
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass DropVelocity-response (<DropVelocity-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DropVelocity-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DropVelocity-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name transformtrack-srv:<DropVelocity-response> is deprecated: use transformtrack-srv:DropVelocity-response instead.")))

(cl:ensure-generic-function 'done-val :lambda-list '(m))
(cl:defmethod done-val ((m <DropVelocity-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader transformtrack-srv:done-val is deprecated.  Use transformtrack-srv:done instead.")
  (done m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DropVelocity-response>) ostream)
  "Serializes a message object of type '<DropVelocity-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'done) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DropVelocity-response>) istream)
  "Deserializes a message object of type '<DropVelocity-response>"
    (cl:setf (cl:slot-value msg 'done) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DropVelocity-response>)))
  "Returns string type for a service object of type '<DropVelocity-response>"
  "transformtrack/DropVelocityResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DropVelocity-response)))
  "Returns string type for a service object of type 'DropVelocity-response"
  "transformtrack/DropVelocityResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DropVelocity-response>)))
  "Returns md5sum for a message object of type '<DropVelocity-response>"
  "2b47bafdc4c00e27a7f0cfd76dc9037e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DropVelocity-response)))
  "Returns md5sum for a message object of type 'DropVelocity-response"
  "2b47bafdc4c00e27a7f0cfd76dc9037e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DropVelocity-response>)))
  "Returns full string definition for message of type '<DropVelocity-response>"
  (cl:format cl:nil "bool done~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DropVelocity-response)))
  "Returns full string definition for message of type 'DropVelocity-response"
  (cl:format cl:nil "bool done~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DropVelocity-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DropVelocity-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DropVelocity-response
    (cl:cons ':done (done msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DropVelocity)))
  'DropVelocity-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DropVelocity)))
  'DropVelocity-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DropVelocity)))
  "Returns string type for a service object of type '<DropVelocity>"
  "transformtrack/DropVelocity")