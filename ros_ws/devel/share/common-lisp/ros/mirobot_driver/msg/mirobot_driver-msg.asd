
(cl:in-package :asdf)

(defsystem "mirobot_driver-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "bot_telemetry" :depends-on ("_package_bot_telemetry"))
    (:file "_package_bot_telemetry" :depends-on ("_package"))
    (:file "wheel_control" :depends-on ("_package_wheel_control"))
    (:file "_package_wheel_control" :depends-on ("_package"))
    (:file "wheel_telemetry" :depends-on ("_package_wheel_telemetry"))
    (:file "_package_wheel_telemetry" :depends-on ("_package"))
  ))