;; Auto-generated. Do not edit!


(when (boundp 'mirobot_driver::wheel_control)
  (if (not (find-package "MIROBOT_DRIVER"))
    (make-package "MIROBOT_DRIVER"))
  (shadow 'wheel_control (find-package "MIROBOT_DRIVER")))
(unless (find-package "MIROBOT_DRIVER::WHEEL_CONTROL")
  (make-package "MIROBOT_DRIVER::WHEEL_CONTROL"))

(in-package "ROS")
;;//! \htmlinclude wheel_control.msg.html


(defclass mirobot_driver::wheel_control
  :super ros::object
  :slots (_dir_l _speed_l _dir_r _speed_r ))

(defmethod mirobot_driver::wheel_control
  (:init
   (&key
    ((:dir_l __dir_l) 0)
    ((:speed_l __speed_l) 0)
    ((:dir_r __dir_r) 0)
    ((:speed_r __speed_r) 0)
    )
   (send-super :init)
   (setq _dir_l (round __dir_l))
   (setq _speed_l (round __speed_l))
   (setq _dir_r (round __dir_r))
   (setq _speed_r (round __speed_r))
   self)
  (:dir_l
   (&optional __dir_l)
   (if __dir_l (setq _dir_l __dir_l)) _dir_l)
  (:speed_l
   (&optional __speed_l)
   (if __speed_l (setq _speed_l __speed_l)) _speed_l)
  (:dir_r
   (&optional __dir_r)
   (if __dir_r (setq _dir_r __dir_r)) _dir_r)
  (:speed_r
   (&optional __speed_r)
   (if __speed_r (setq _speed_r __speed_r)) _speed_r)
  (:serialization-length
   ()
   (+
    ;; uint8 _dir_l
    1
    ;; uint8 _speed_l
    1
    ;; uint8 _dir_r
    1
    ;; uint8 _speed_r
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _dir_l
       (write-byte _dir_l s)
     ;; uint8 _speed_l
       (write-byte _speed_l s)
     ;; uint8 _dir_r
       (write-byte _dir_r s)
     ;; uint8 _speed_r
       (write-byte _speed_r s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _dir_l
     (setq _dir_l (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _speed_l
     (setq _speed_l (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _dir_r
     (setq _dir_r (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _speed_r
     (setq _speed_r (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get mirobot_driver::wheel_control :md5sum-) "7bf657bb437a3a998dcf9c0bf0cd51fa")
(setf (get mirobot_driver::wheel_control :datatype-) "mirobot_driver/wheel_control")
(setf (get mirobot_driver::wheel_control :definition-)
      "uint8 dir_l
uint8 speed_l
uint8 dir_r
uint8 speed_r

")



(provide :mirobot_driver/wheel_control "7bf657bb437a3a998dcf9c0bf0cd51fa")


