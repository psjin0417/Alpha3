; Auto-generated. Do not edit!


(cl:in-package package-msg)


;//! \htmlinclude gnss.msg.html

(cl:defclass <gnss> (roslisp-msg-protocol:ros-message)
  ((East
    :reader East
    :initarg :East
    :type cl:float
    :initform 0.0)
   (North
    :reader North
    :initarg :North
    :type cl:float
    :initform 0.0)
   (Yaw
    :reader Yaw
    :initarg :Yaw
    :type cl:float
    :initform 0.0))
)

(cl:defclass gnss (<gnss>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gnss>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gnss)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name package-msg:<gnss> is deprecated: use package-msg:gnss instead.")))

(cl:ensure-generic-function 'East-val :lambda-list '(m))
(cl:defmethod East-val ((m <gnss>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader package-msg:East-val is deprecated.  Use package-msg:East instead.")
  (East m))

(cl:ensure-generic-function 'North-val :lambda-list '(m))
(cl:defmethod North-val ((m <gnss>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader package-msg:North-val is deprecated.  Use package-msg:North instead.")
  (North m))

(cl:ensure-generic-function 'Yaw-val :lambda-list '(m))
(cl:defmethod Yaw-val ((m <gnss>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader package-msg:Yaw-val is deprecated.  Use package-msg:Yaw instead.")
  (Yaw m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gnss>) ostream)
  "Serializes a message object of type '<gnss>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'East))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'North))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'Yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gnss>) istream)
  "Deserializes a message object of type '<gnss>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'East) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'North) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Yaw) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gnss>)))
  "Returns string type for a message object of type '<gnss>"
  "package/gnss")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gnss)))
  "Returns string type for a message object of type 'gnss"
  "package/gnss")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gnss>)))
  "Returns md5sum for a message object of type '<gnss>"
  "786bb79528800c839905020a7bf7d1ae")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gnss)))
  "Returns md5sum for a message object of type 'gnss"
  "786bb79528800c839905020a7bf7d1ae")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gnss>)))
  "Returns full string definition for message of type '<gnss>"
  (cl:format cl:nil "float64 East~%float64 North~%float64 Yaw~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gnss)))
  "Returns full string definition for message of type 'gnss"
  (cl:format cl:nil "float64 East~%float64 North~%float64 Yaw~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gnss>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gnss>))
  "Converts a ROS message object to a list"
  (cl:list 'gnss
    (cl:cons ':East (East msg))
    (cl:cons ':North (North msg))
    (cl:cons ':Yaw (Yaw msg))
))
