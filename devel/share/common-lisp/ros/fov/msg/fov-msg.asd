
(cl:in-package :asdf)

(defsystem "fov-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "FishState" :depends-on ("_package_FishState"))
    (:file "_package_FishState" :depends-on ("_package"))
  ))