
(cl:in-package :asdf)

(defsystem "bot_hal-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "wheel_control" :depends-on ("_package_wheel_control"))
    (:file "_package_wheel_control" :depends-on ("_package"))
  ))