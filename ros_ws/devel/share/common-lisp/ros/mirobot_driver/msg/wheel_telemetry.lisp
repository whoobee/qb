; Auto-generated. Do not edit!


(cl:in-package mirobot_driver-msg)


;//! \htmlinclude wheel_telemetry.msg.html

(cl:defclass <wheel_telemetry> (roslisp-msg-protocol:ros-message)
  ((speed_l
    :reader speed_l
    :initarg :speed_l
    :type cl:fixnum
    :initform 0)
   (speed_r
    :reader speed_r
    :initarg :speed_r
    :type cl:fixnum
    :initform 0))
)

(cl:defclass wheel_telemetry (<wheel_telemetry>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <wheel_telemetry>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'wheel_telemetry)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mirobot_driver-msg:<wheel_telemetry> is deprecated: use mirobot_driver-msg:wheel_telemetry instead.")))

(cl:ensure-generic-function 'speed_l-val :lambda-list '(m))
(cl:defmethod speed_l-val ((m <wheel_telemetry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirobot_driver-msg:speed_l-val is deprecated.  Use mirobot_driver-msg:speed_l instead.")
  (speed_l m))

(cl:ensure-generic-function 'speed_r-val :lambda-list '(m))
(cl:defmethod speed_r-val ((m <wheel_telemetry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirobot_driver-msg:speed_r-val is deprecated.  Use mirobot_driver-msg:speed_r instead.")
  (speed_r m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <wheel_telemetry>) ostream)
  "Serializes a message object of type '<wheel_telemetry>"
  (cl:let* ((signed (cl:slot-value msg 'speed_l)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'speed_r)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <wheel_telemetry>) istream)
  "Deserializes a message object of type '<wheel_telemetry>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'speed_l) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'speed_r) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<wheel_telemetry>)))
  "Returns string type for a message object of type '<wheel_telemetry>"
  "mirobot_driver/wheel_telemetry")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'wheel_telemetry)))
  "Returns string type for a message object of type 'wheel_telemetry"
  "mirobot_driver/wheel_telemetry")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<wheel_telemetry>)))
  "Returns md5sum for a message object of type '<wheel_telemetry>"
  "6a13c855fd502a6ed208724a11e8020c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'wheel_telemetry)))
  "Returns md5sum for a message object of type 'wheel_telemetry"
  "6a13c855fd502a6ed208724a11e8020c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<wheel_telemetry>)))
  "Returns full string definition for message of type '<wheel_telemetry>"
  (cl:format cl:nil "int16 speed_l~%int16 speed_r~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'wheel_telemetry)))
  "Returns full string definition for message of type 'wheel_telemetry"
  (cl:format cl:nil "int16 speed_l~%int16 speed_r~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <wheel_telemetry>))
  (cl:+ 0
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <wheel_telemetry>))
  "Converts a ROS message object to a list"
  (cl:list 'wheel_telemetry
    (cl:cons ':speed_l (speed_l msg))
    (cl:cons ':speed_r (speed_r msg))
))
