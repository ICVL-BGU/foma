
(cl:in-package :asdf)

(defsystem "fov-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "FishState" :depends-on ("_package_FishState"))
    (:file "_package_FishState" :depends-on ("_package"))
    (:file "FomaLocation" :depends-on ("_package_FomaLocation"))
    (:file "_package_FomaLocation" :depends-on ("_package"))
  ))