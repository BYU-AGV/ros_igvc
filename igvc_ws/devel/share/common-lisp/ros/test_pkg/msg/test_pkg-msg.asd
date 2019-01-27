
(cl:in-package :asdf)

(defsystem "test_pkg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "coord" :depends-on ("_package_coord"))
    (:file "_package_coord" :depends-on ("_package"))
    (:file "coord" :depends-on ("_package_coord"))
    (:file "_package_coord" :depends-on ("_package"))
  ))