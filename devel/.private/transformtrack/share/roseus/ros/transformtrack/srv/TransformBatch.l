;; Auto-generated. Do not edit!


(when (boundp 'transformtrack::TransformBatch)
  (if (not (find-package "TRANSFORMTRACK"))
    (make-package "TRANSFORMTRACK"))
  (shadow 'TransformBatch (find-package "TRANSFORMTRACK")))
(unless (find-package "TRANSFORMTRACK::TRANSFORMBATCH")
  (make-package "TRANSFORMTRACK::TRANSFORMBATCH"))
(unless (find-package "TRANSFORMTRACK::TRANSFORMBATCHREQUEST")
  (make-package "TRANSFORMTRACK::TRANSFORMBATCHREQUEST"))
(unless (find-package "TRANSFORMTRACK::TRANSFORMBATCHRESPONSE")
  (make-package "TRANSFORMTRACK::TRANSFORMBATCHRESPONSE"))

(in-package "ROS")



(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass transformtrack::TransformBatchRequest
  :super ros::object
  :slots (_start_times _end_time ))

(defmethod transformtrack::TransformBatchRequest
  (:init
   (&key
    ((:start_times __start_times) (let (r) (dotimes (i 0) (push (instance ros::time :init) r)) r))
    ((:end_time __end_time) (instance ros::time :init))
    )
   (send-super :init)
   (setq _start_times __start_times)
   (setq _end_time __end_time)
   self)
  (:start_times
   (&optional __start_times)
   (if __start_times (setq _start_times __start_times)) _start_times)
  (:end_time
   (&optional __end_time)
   (if __end_time (setq _end_time __end_time)) _end_time)
  (:serialization-length
   ()
   (+
    ;; time[] _start_times
    (* 8    (length _start_times)) 4
    ;; time _end_time
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time[] _start_times
     (write-long (length _start_times) s)
     (dotimes (i (length _start_times))
       (write-long (send (elt _start_times i) :sec) s) (write-long (send (elt _start_times i) :nsec) s)
       )
     ;; time _end_time
       (write-long (send _end_time :sec) s) (write-long (send _end_time :nsec) s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time[] _start_times
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _start_times (let (r) (dotimes (i n) (push (instance ros::time :init) r)) r))
     (dolist (elem- _start_times)
     (send elem- :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send elem- :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; time _end_time
     (send _end_time :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _end_time :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass transformtrack::TransformBatchResponse
  :super ros::object
  :slots (_transforms _distances ))

(defmethod transformtrack::TransformBatchResponse
  (:init
   (&key
    ((:transforms __transforms) (instance std_msgs::Float64MultiArray :init))
    ((:distances __distances) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _transforms __transforms)
   (setq _distances __distances)
   self)
  (:transforms
   (&rest __transforms)
   (if (keywordp (car __transforms))
       (send* _transforms __transforms)
     (progn
       (if __transforms (setq _transforms (car __transforms)))
       _transforms)))
  (:distances
   (&optional __distances)
   (if __distances (setq _distances __distances)) _distances)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Float64MultiArray _transforms
    (send _transforms :serialization-length)
    ;; float64[] _distances
    (* 8    (length _distances)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Float64MultiArray _transforms
       (send _transforms :serialize s)
     ;; float64[] _distances
     (write-long (length _distances) s)
     (dotimes (i (length _distances))
       (sys::poke (elt _distances i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Float64MultiArray _transforms
     (send _transforms :deserialize buf ptr-) (incf ptr- (send _transforms :serialization-length))
   ;; float64[] _distances
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _distances (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _distances i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(defclass transformtrack::TransformBatch
  :super ros::object
  :slots ())

(setf (get transformtrack::TransformBatch :md5sum-) "d17c54b06f263044d75d6d03617a9ba1")
(setf (get transformtrack::TransformBatch :datatype-) "transformtrack/TransformBatch")
(setf (get transformtrack::TransformBatch :request) transformtrack::TransformBatchRequest)
(setf (get transformtrack::TransformBatch :response) transformtrack::TransformBatchResponse)

(defmethod transformtrack::TransformBatchRequest
  (:response () (instance transformtrack::TransformBatchResponse :init)))

(setf (get transformtrack::TransformBatchRequest :md5sum-) "d17c54b06f263044d75d6d03617a9ba1")
(setf (get transformtrack::TransformBatchRequest :datatype-) "transformtrack/TransformBatchRequest")
(setf (get transformtrack::TransformBatchRequest :definition-)
      "time[] start_times
time end_time
---
std_msgs/Float64MultiArray transforms
float64[] distances

================================================================================
MSG: std_msgs/Float64MultiArray
# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
float64[]         data          # array of data


================================================================================
MSG: std_msgs/MultiArrayLayout
# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
# 
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = \"height\"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = \"width\"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = \"channel\"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
")

(setf (get transformtrack::TransformBatchResponse :md5sum-) "d17c54b06f263044d75d6d03617a9ba1")
(setf (get transformtrack::TransformBatchResponse :datatype-) "transformtrack/TransformBatchResponse")
(setf (get transformtrack::TransformBatchResponse :definition-)
      "time[] start_times
time end_time
---
std_msgs/Float64MultiArray transforms
float64[] distances

================================================================================
MSG: std_msgs/Float64MultiArray
# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
float64[]         data          # array of data


================================================================================
MSG: std_msgs/MultiArrayLayout
# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
# 
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = \"height\"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = \"width\"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = \"channel\"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
")



(provide :transformtrack/TransformBatch "d17c54b06f263044d75d6d03617a9ba1")


