;; Auto-generated. Do not edit!


(when (boundp 'mirobot_driver::bot_telemetry)
  (if (not (find-package "MIROBOT_DRIVER"))
    (make-package "MIROBOT_DRIVER"))
  (shadow 'bot_telemetry (find-package "MIROBOT_DRIVER")))
(unless (find-package "MIROBOT_DRIVER::BOT_TELEMETRY")
  (make-package "MIROBOT_DRIVER::BOT_TELEMETRY"))

(in-package "ROS")
;;//! \htmlinclude bot_telemetry.msg.html


(defclass mirobot_driver::bot_telemetry
  :super ros::object
  :slots (_id _gyro_x _gyro_y _gyro_z _acc_x _acc_y _acc_z ))

(defmethod mirobot_driver::bot_telemetry
  (:init
   (&key
    ((:id __id) 0)
    ((:gyro_x __gyro_x) 0)
    ((:gyro_y __gyro_y) 0)
    ((:gyro_z __gyro_z) 0)
    ((:acc_x __acc_x) 0)
    ((:acc_y __acc_y) 0)
    ((:acc_z __acc_z) 0)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _gyro_x (round __gyro_x))
   (setq _gyro_y (round __gyro_y))
   (setq _gyro_z (round __gyro_z))
   (setq _acc_x (round __acc_x))
   (setq _acc_y (round __acc_y))
   (setq _acc_z (round __acc_z))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:gyro_x
   (&optional __gyro_x)
   (if __gyro_x (setq _gyro_x __gyro_x)) _gyro_x)
  (:gyro_y
   (&optional __gyro_y)
   (if __gyro_y (setq _gyro_y __gyro_y)) _gyro_y)
  (:gyro_z
   (&optional __gyro_z)
   (if __gyro_z (setq _gyro_z __gyro_z)) _gyro_z)
  (:acc_x
   (&optional __acc_x)
   (if __acc_x (setq _acc_x __acc_x)) _acc_x)
  (:acc_y
   (&optional __acc_y)
   (if __acc_y (setq _acc_y __acc_y)) _acc_y)
  (:acc_z
   (&optional __acc_z)
   (if __acc_z (setq _acc_z __acc_z)) _acc_z)
  (:serialization-length
   ()
   (+
    ;; uint8 _id
    1
    ;; int32 _gyro_x
    4
    ;; int32 _gyro_y
    4
    ;; int32 _gyro_z
    4
    ;; int32 _acc_x
    4
    ;; int32 _acc_y
    4
    ;; int32 _acc_z
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _id
       (write-byte _id s)
     ;; int32 _gyro_x
       (write-long _gyro_x s)
     ;; int32 _gyro_y
       (write-long _gyro_y s)
     ;; int32 _gyro_z
       (write-long _gyro_z s)
     ;; int32 _acc_x
       (write-long _acc_x s)
     ;; int32 _acc_y
       (write-long _acc_y s)
     ;; int32 _acc_z
       (write-long _acc_z s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _id
     (setq _id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; int32 _gyro_x
     (setq _gyro_x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _gyro_y
     (setq _gyro_y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _gyro_z
     (setq _gyro_z (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _acc_x
     (setq _acc_x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _acc_y
     (setq _acc_y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _acc_z
     (setq _acc_z (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get mirobot_driver::bot_telemetry :md5sum-) "c924c03de4741fb4b40f30ce9f8a6fe4")
(setf (get mirobot_driver::bot_telemetry :datatype-) "mirobot_driver/bot_telemetry")
(setf (get mirobot_driver::bot_telemetry :definition-)
      "uint8 id
int32 gyro_x
int32 gyro_y
int32 gyro_z
int32 acc_x
int32 acc_y
int32 acc_z

")



(provide :mirobot_driver/bot_telemetry "c924c03de4741fb4b40f30ce9f8a6fe4")


