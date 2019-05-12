;; Auto-generated. Do not edit!


(when (boundp 'bot_hal::wheel_control)
  (if (not (find-package "BOT_HAL"))
    (make-package "BOT_HAL"))
  (shadow 'wheel_control (find-package "BOT_HAL")))
(unless (find-package "BOT_HAL::WHEEL_CONTROL")
  (make-package "BOT_HAL::WHEEL_CONTROL"))

(in-package "ROS")
;;//! \htmlinclude wheel_control.msg.html


(defclass bot_hal::wheel_control
  :super ros::object
  :slots (_id _speed ))

(defmethod bot_hal::wheel_control
  (:init
   (&key
    ((:id __id) 0)
    ((:speed __speed) 0)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _speed (round __speed))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:serialization-length
   ()
   (+
    ;; int32 _id
    4
    ;; int32 _speed
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _id
       (write-long _id s)
     ;; int32 _speed
       (write-long _speed s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _speed
     (setq _speed (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get bot_hal::wheel_control :md5sum-) "6844aaf90d2570626588414c58ddac1a")
(setf (get bot_hal::wheel_control :datatype-) "bot_hal/wheel_control")
(setf (get bot_hal::wheel_control :definition-)
      "int32 id
int32 speed

")



(provide :bot_hal/wheel_control "6844aaf90d2570626588414c58ddac1a")


