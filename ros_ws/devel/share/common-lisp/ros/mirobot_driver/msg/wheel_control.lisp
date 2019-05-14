; Auto-generated. Do not edit!


(cl:in-package mirobot_driver-msg)


;//! \htmlinclude wheel_control.msg.html

(cl:defclass <wheel_control> (roslisp-msg-protocol:ros-message)
  ((dir_l
    :reader dir_l
    :initarg :dir_l
    :type cl:fixnum
    :initform 0)
   (speed_l
    :reader speed_l
    :initarg :speed_l
    :type cl:fixnum
    :initform 0)
   (dir_r
    :reader dir_r
    :initarg :dir_r
    :type cl:fixnum
    :initform 0)
   (speed_r
    :reader speed_r
    :initarg :speed_r
    :type cl:fixnum
    :initform 0))
)

(cl:defclass wheel_control (<wheel_control>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <wheel_control>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'wheel_control)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mirobot_driver-msg:<wheel_control> is deprecated: use mirobot_driver-msg:wheel_control instead.")))

(cl:ensure-generic-function 'dir_l-val :lambda-list '(m))
(cl:defmethod dir_l-val ((m <wheel_control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirobot_driver-msg:dir_l-val is deprecated.  Use mirobot_driver-msg:dir_l instead.")
  (dir_l m))

(cl:ensure-generic-function 'speed_l-val :lambda-list '(m))
(cl:defmethod speed_l-val ((m <wheel_control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirobot_driver-msg:speed_l-val is deprecated.  Use mirobot_driver-msg:speed_l instead.")
  (speed_l m))

(cl:ensure-generic-function 'dir_r-val :lambda-list '(m))
(cl:defmethod dir_r-val ((m <wheel_control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirobot_driver-msg:dir_r-val is deprecated.  Use mirobot_driver-msg:dir_r instead.")
  (dir_r m))

(cl:ensure-generic-function 'speed_r-val :lambda-list '(m))
(cl:defmethod speed_r-val ((m <wheel_control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirobot_driver-msg:speed_r-val is deprecated.  Use mirobot_driver-msg:speed_r instead.")
  (speed_r m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <wheel_control>) ostream)
  "Serializes a message object of type '<wheel_control>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dir_l)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'speed_l)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dir_r)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'speed_r)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <wheel_control>) istream)
  "Deserializes a message object of type '<wheel_control>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dir_l)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'speed_l)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dir_r)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'speed_r)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<wheel_control>)))
  "Returns string type for a message object of type '<wheel_control>"
  "mirobot_driver/wheel_control")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'wheel_control)))
  "Returns string type for a message object of type 'wheel_control"
  "mirobot_driver/wheel_control")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<wheel_control>)))
  "Returns md5sum for a message object of type '<wheel_control>"
  "7bf657bb437a3a998dcf9c0bf0cd51fa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'wheel_control)))
  "Returns md5sum for a message object of type 'wheel_control"
  "7bf657bb437a3a998dcf9c0bf0cd51fa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<wheel_control>)))
  "Returns full string definition for message of type '<wheel_control>"
  (cl:format cl:nil "uint8 dir_l~%uint8 speed_l~%uint8 dir_r~%uint8 speed_r~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'wheel_control)))
  "Returns full string definition for message of type 'wheel_control"
  (cl:format cl:nil "uint8 dir_l~%uint8 speed_l~%uint8 dir_r~%uint8 speed_r~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <wheel_control>))
  (cl:+ 0
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <wheel_control>))
  "Converts a ROS message object to a list"
  (cl:list 'wheel_control
    (cl:cons ':dir_l (dir_l msg))
    (cl:cons ':speed_l (speed_l msg))
    (cl:cons ':dir_r (dir_r msg))
    (cl:cons ':speed_r (speed_r msg))
))
