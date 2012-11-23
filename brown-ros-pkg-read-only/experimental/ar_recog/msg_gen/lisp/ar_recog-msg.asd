
(cl:in-package :asdf)

(defsystem "ar_recog-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Tags" :depends-on ("_package_Tags"))
    (:file "_package_Tags" :depends-on ("_package"))
    (:file "Tag" :depends-on ("_package_Tag"))
    (:file "_package_Tag" :depends-on ("_package"))
  ))