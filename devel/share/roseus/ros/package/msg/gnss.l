;; Auto-generated. Do not edit!


(when (boundp 'package::gnss)
  (if (not (find-package "PACKAGE"))
    (make-package "PACKAGE"))
  (shadow 'gnss (find-package "PACKAGE")))
(unless (find-package "PACKAGE::GNSS")
  (make-package "PACKAGE::GNSS"))

(in-package "ROS")
;;//! \htmlinclude gnss.msg.html


(defclass package::gnss
  :super ros::object
  :slots (_East _North _Yaw ))

(defmethod package::gnss
  (:init
   (&key
    ((:East __East) 0.0)
    ((:North __North) 0.0)
    ((:Yaw __Yaw) 0.0)
    )
   (send-super :init)
   (setq _East (float __East))
   (setq _North (float __North))
   (setq _Yaw (float __Yaw))
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
  (:serialization-length
   ()
   (+
    ;; float64 _East
    8
    ;; float64 _North
    8
    ;; float64 _Yaw
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
   ;;
   self)
  )

(setf (get package::gnss :md5sum-) "786bb79528800c839905020a7bf7d1ae")
(setf (get package::gnss :datatype-) "package/gnss")
(setf (get package::gnss :definition-)
      "float64 East
float64 North
float64 Yaw

")



(provide :package/gnss "786bb79528800c839905020a7bf7d1ae")


