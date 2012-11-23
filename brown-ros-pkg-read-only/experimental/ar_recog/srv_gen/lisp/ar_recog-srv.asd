
(cl:in-package :asdf)

(defsystem "ar_recog-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "CalibrateDistance" :depends-on ("_package_CalibrateDistance"))
    (:file "_package_CalibrateDistance" :depends-on ("_package"))
  ))