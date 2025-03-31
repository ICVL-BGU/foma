;; Auto-generated. Do not edit!


(when (boundp 'fov::FomaLocation)
  (if (not (find-package "FOV"))
    (make-package "FOV"))
  (shadow 'FomaLocation (find-package "FOV")))
(unless (find-package "FOV::FOMALOCATION")
  (make-package "FOV::FOMALOCATION"))

(in-package "ROS")
;;//! \htmlinclude FomaLocation.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass fov::FomaLocation
  :super ros::object
  :slots (_image _world ))

(defmethod fov::FomaLocation
  (:init
   (&key
    ((:image __image) (instance geometry_msgs::Point :init))
    ((:world __world) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _image __image)
   (setq _world __world)
   self)
  (:image
   (&rest __image)
   (if (keywordp (car __image))
       (send* _image __image)
     (progn
       (if __image (setq _image (car __image)))
       _image)))
  (:world
   (&rest __world)
   (if (keywordp (car __world))
       (send* _world __world)
     (progn
       (if __world (setq _world (car __world)))
       _world)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _image
    (send _image :serialization-length)
    ;; geometry_msgs/Point _world
    (send _world :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _image
       (send _image :serialize s)
     ;; geometry_msgs/Point _world
       (send _world :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _image
     (send _image :deserialize buf ptr-) (incf ptr- (send _image :serialization-length))
   ;; geometry_msgs/Point _world
     (send _world :deserialize buf ptr-) (incf ptr- (send _world :serialization-length))
   ;;
   self)
  )

(setf (get fov::FomaLocation :md5sum-) "5b07c96efdecb7282bf61c35bd56a0d2")
(setf (get fov::FomaLocation :datatype-) "fov/FomaLocation")
(setf (get fov::FomaLocation :definition-)
      "geometry_msgs/Point image
geometry_msgs/Point world
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :fov/FomaLocation "5b07c96efdecb7282bf61c35bd56a0d2")


