
(cl:in-package :asdf)

(defsystem "plutodrone-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "PlutoPilot" :depends-on ("_package_PlutoPilot"))
    (:file "_package_PlutoPilot" :depends-on ("_package"))
    (:file "aruco" :depends-on ("_package_aruco"))
    (:file "_package_aruco" :depends-on ("_package"))
  ))