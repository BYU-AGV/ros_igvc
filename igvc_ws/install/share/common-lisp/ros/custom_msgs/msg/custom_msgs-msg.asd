
(cl:in-package :asdf)

(defsystem "custom_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "coord" :depends-on ("_package_coord"))
    (:file "_package_coord" :depends-on ("_package"))
    (:file "coord" :depends-on ("_package_coord"))
    (:file "_package_coord" :depends-on ("_package"))
  ))