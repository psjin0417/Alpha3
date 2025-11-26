
(cl:in-package :asdf)

(defsystem "package-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "gnss" :depends-on ("_package_gnss"))
    (:file "_package_gnss" :depends-on ("_package"))
    (:file "object" :depends-on ("_package_object"))
    (:file "_package_object" :depends-on ("_package"))
  ))