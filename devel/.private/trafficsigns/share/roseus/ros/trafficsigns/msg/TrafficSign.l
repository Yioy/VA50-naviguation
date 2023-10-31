;; Auto-generated. Do not edit!


(when (boundp 'trafficsigns::TrafficSign)
  (if (not (find-package "TRAFFICSIGNS"))
    (make-package "TRAFFICSIGNS"))
  (shadow 'TrafficSign (find-package "TRAFFICSIGNS")))
(unless (find-package "TRAFFICSIGNS::TRAFFICSIGN")
  (make-package "TRAFFICSIGNS::TRAFFICSIGN"))

(in-package "ROS")
;;//! \htmlinclude TrafficSign.msg.html


(defclass trafficsigns::TrafficSign
  :super ros::object
  :slots (_category _type _x _y _z _confidence ))

(defmethod trafficsigns::TrafficSign
  (:init
   (&key
    ((:category __category) "")
    ((:type __type) "")
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    ((:confidence __confidence) 0.0)
    )
   (send-super :init)
   (setq _category (string __category))
   (setq _type (string __type))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   (setq _confidence (float __confidence))
   self)
  (:category
   (&optional __category)
   (if __category (setq _category __category)) _category)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:confidence
   (&optional __confidence)
   (if __confidence (setq _confidence __confidence)) _confidence)
  (:serialization-length
   ()
   (+
    ;; string _category
    4 (length _category)
    ;; string _type
    4 (length _type)
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _z
    4
    ;; float32 _confidence
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _category
       (write-long (length _category) s) (princ _category s)
     ;; string _type
       (write-long (length _type) s) (princ _type s)
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z
       (sys::poke _z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _confidence
       (sys::poke _confidence (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _category
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _category (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z
     (setq _z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _confidence
     (setq _confidence (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get trafficsigns::TrafficSign :md5sum-) "cce144ea2124a75281ff7e5d5f7d931e")
(setf (get trafficsigns::TrafficSign :datatype-) "trafficsigns/TrafficSign")
(setf (get trafficsigns::TrafficSign :definition-)
      "string category
string type
float32 x
float32 y
float32 z
float32 confidence
")



(provide :trafficsigns/TrafficSign "cce144ea2124a75281ff7e5d5f7d931e")


