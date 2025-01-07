
(cl:in-package :asdf)

(defsystem "fov-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Check" :depends-on ("_package_Check"))
    (:file "_package_Check" :depends-on ("_package"))
    (:file "Coordinate" :depends-on ("_package_Coordinate"))
    (:file "_package_Coordinate" :depends-on ("_package"))
    (:file "EPDImage" :depends-on ("_package_EPDImage"))
    (:file "_package_EPDImage" :depends-on ("_package"))
    (:file "Light" :depends-on ("_package_Light"))
    (:file "_package_Light" :depends-on ("_package"))
  ))