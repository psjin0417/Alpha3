;; Auto-generated. Do not edit!


(when (boundp 'package::object)
  (if (not (find-package "PACKAGE"))
    (make-package "PACKAGE"))
  (shadow 'object (find-package "PACKAGE")))
(unless (find-package "PACKAGE::OBJECT")
  (make-package "PACKAGE::OBJECT"))

(in-package "ROS")
;;//! \htmlinclude object.msg.html


(defclass package::object
  :super ros::object
  :slots (_East _North _Yaw _Vel ))

(defmethod package::object
  (:init
   (&key
    ((:East __East) 0.0)
    ((:North __North) 0.0)
    ((:Yaw __Yaw) 0.0)
    ((:Vel __Vel) 0.0)
    )
   (send-super :init)
   (setq _East (float __East))
   (setq _North (float __North))
   (setq _Yaw (float __Yaw))
   (setq _Vel (float __Vel))
   self)
  (:East
   (&optional __East)
   (if __East (setq _East __East)) _East)
  (:North
   (&optional __North)
   (if __North (setq _North __North)) _North)
  (:Yaw
   (&optional __Yaw)
   (if __Yaw (setq _Yaw __Yaw)) _Yaw)
  (:Vel
   (&optional __Vel)
   (if __Vel (setq _Vel __Vel)) _Vel)
  (:serialization-length
   ()
   (+
    ;; float64 _East
    8
    ;; float64 _North
    8
    ;; float64 _Yaw
    8
    ;; float64 _Vel
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _East
       (sys::poke _East (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _North
       (sys::poke _North (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Yaw
       (sys::poke _Yaw (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Vel
       (sys::poke _Vel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _East
     (setq _East (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _North
     (setq _North (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Yaw
     (setq _Yaw (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Vel
     (setq _Vel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get package::object :md5sum-) "1738d351724b3c4d39cc9127cdfa9d03")
(setf (get package::object :datatype-) "package/object")
(setf (get package::object :definition-)
      "float64 East
float64 North
float64 Yaw
float64 Vel
")



(provide :package/object "1738d351724b3c4d39cc9127cdfa9d03")


