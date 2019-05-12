;; Auto-generated. Do not edit!


(when (boundp 'mirobot_driver::wheel_telemetry)
  (if (not (find-package "MIROBOT_DRIVER"))
    (make-package "MIROBOT_DRIVER"))
  (shadow 'wheel_telemetry (find-package "MIROBOT_DRIVER")))
(unless (find-package "MIROBOT_DRIVER::WHEEL_TELEMETRY")
  (make-package "MIROBOT_DRIVER::WHEEL_TELEMETRY"))

(in-package "ROS")
;;//! \htmlinclude wheel_telemetry.msg.html


(defclass mirobot_driver::wheel_telemetry
  :super ros::object
  :slots (_speed_l _speed_r ))

(defmethod mirobot_driver::wheel_telemetry
  (:init
   (&key
    ((:speed_l __speed_l) 0)
    ((:speed_r __speed_r) 0)
    )
   (send-super :init)
   (setq _speed_l (round __speed_l))
   (setq _speed_r (round __speed_r))
   self)
  (:speed_l
   (&optional __speed_l)
   (if __speed_l (setq _speed_l __speed_l)) _speed_l)
  (:speed_r
   (&optional __speed_r)
   (if __speed_r (setq _speed_r __speed_r)) _speed_r)
  (:serialization-length
   ()
   (+
    ;; int16 _speed_l
    2
    ;; int16 _speed_r
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _speed_l
       (write-word _speed_l s)
     ;; int16 _speed_r
       (write-word _speed_r s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _speed_l
     (setq _speed_l (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _speed_r
     (setq _speed_r (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get mirobot_driver::wheel_telemetry :md5sum-) "6a13c855fd502a6ed208724a11e8020c")
(setf (get mirobot_driver::wheel_telemetry :datatype-) "mirobot_driver/wheel_telemetry")
(setf (get mirobot_driver::wheel_telemetry :definition-)
      "int16 speed_l
int16 speed_r

")



(provide :mirobot_driver/wheel_telemetry "6a13c855fd502a6ed208724a11e8020c")


