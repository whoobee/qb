; Auto-generated. Do not edit!


(cl:in-package bot_hal-msg)


;//! \htmlinclude wheel_control.msg.html

(cl:defclass <wheel_control> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (speed
    :reader speed
    :initarg :speed
    :type cl:integer
    :initform 0))
)

(cl:defclass wheel_control (<wheel_control>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <wheel_control>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'wheel_control)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bot_hal-msg:<wheel_control> is deprecated: use bot_hal-msg:wheel_control instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <wheel_control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bot_hal-msg:id-val is deprecated.  Use bot_hal-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <wheel_control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bot_hal-msg:speed-val is deprecated.  Use bot_hal-msg:speed instead.")
  (speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <wheel_control>) ostream)
  "Serializes a message object of type '<wheel_control>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <wheel_control>) istream)
  "Deserializes a message object of type '<wheel_control>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'speed) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<wheel_control>)))
  "Returns string type for a message object of type '<wheel_control>"
  "bot_hal/wheel_control")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'wheel_control)))
  "Returns string type for a message object of type 'wheel_control"
  "bot_hal/wheel_control")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<wheel_control>)))
  "Returns md5sum for a message object of type '<wheel_control>"
  "6844aaf90d2570626588414c58ddac1a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'wheel_control)))
  "Returns md5sum for a message object of type 'wheel_control"
  "6844aaf90d2570626588414c58ddac1a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<wheel_control>)))
  "Returns full string definition for message of type '<wheel_control>"
  (cl:format cl:nil "int32 id~%int32 speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'wheel_control)))
  "Returns full string definition for message of type 'wheel_control"
  (cl:format cl:nil "int32 id~%int32 speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <wheel_control>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <wheel_control>))
  "Converts a ROS message object to a list"
  (cl:list 'wheel_control
    (cl:cons ':id (id msg))
    (cl:cons ':speed (speed msg))
))
