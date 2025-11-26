; Auto-generated. Do not edit!


(cl:in-package package-msg)


;//! \htmlinclude object.msg.html

(cl:defclass <object> (roslisp-msg-protocol:ros-message)
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
    :initform 0.0)
   (Vel
    :reader Vel
    :initarg :Vel
    :type cl:float
    :initform 0.0))
)

(cl:defclass object (<object>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <object>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'object)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name package-msg:<object> is deprecated: use package-msg:object instead.")))

(cl:ensure-generic-function 'East-val :lambda-list '(m))
(cl:defmethod East-val ((m <object>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader package-msg:East-val is deprecated.  Use package-msg:East instead.")
  (East m))

(cl:ensure-generic-function 'North-val :lambda-list '(m))
(cl:defmethod North-val ((m <object>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader package-msg:North-val is deprecated.  Use package-msg:North instead.")
  (North m))

(cl:ensure-generic-function 'Yaw-val :lambda-list '(m))
(cl:defmethod Yaw-val ((m <object>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader package-msg:Yaw-val is deprecated.  Use package-msg:Yaw instead.")
  (Yaw m))

(cl:ensure-generic-function 'Vel-val :lambda-list '(m))
(cl:defmethod Vel-val ((m <object>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader package-msg:Vel-val is deprecated.  Use package-msg:Vel instead.")
  (Vel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <object>) ostream)
  "Serializes a message object of type '<object>"
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
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'Vel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <object>) istream)
  "Deserializes a message object of type '<object>"
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
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Vel) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<object>)))
  "Returns string type for a message object of type '<object>"
  "package/object")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'object)))
  "Returns string type for a message object of type 'object"
  "package/object")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<object>)))
  "Returns md5sum for a message object of type '<object>"
  "1738d351724b3c4d39cc9127cdfa9d03")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'object)))
  "Returns md5sum for a message object of type 'object"
  "1738d351724b3c4d39cc9127cdfa9d03")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<object>)))
  "Returns full string definition for message of type '<object>"
  (cl:format cl:nil "float64 East~%float64 North~%float64 Yaw~%float64 Vel~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'object)))
  "Returns full string definition for message of type 'object"
  (cl:format cl:nil "float64 East~%float64 North~%float64 Yaw~%float64 Vel~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <object>))
  (cl:+ 0
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <object>))
  "Converts a ROS message object to a list"
  (cl:list 'object
    (cl:cons ':East (East msg))
    (cl:cons ':North (North msg))
    (cl:cons ':Yaw (Yaw msg))
    (cl:cons ':Vel (Vel msg))
))
