
(cl:in-package :asdf)

(defsystem "youbot_imarker-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "diffval" :depends-on ("_package_diffval"))
    (:file "_package_diffval" :depends-on ("_package"))
  ))