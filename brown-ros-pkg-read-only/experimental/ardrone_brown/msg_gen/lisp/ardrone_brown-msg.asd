
(cl:in-package :asdf)

(defsystem "ardrone_brown-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Navdata" :depends-on ("_package_Navdata"))
    (:file "_package_Navdata" :depends-on ("_package"))
  ))