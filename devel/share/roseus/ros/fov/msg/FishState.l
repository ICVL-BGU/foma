;; Auto-generated. Do not edit!


(when (boundp 'fov::FishState)
  (if (not (find-package "FOV"))
    (make-package "FOV"))
  (shadow 'FishState (find-package "FOV")))
(unless (find-package "FOV::FISHSTATE")
  (make-package "FOV::FISHSTATE"))

(in-package "ROS")
;;//! \htmlinclude FishState.msg.html


(defclass fov::FishState
  :super ros::object
  :slots (_direction _x _y ))

(defmethod fov::FishState
  (:init
   (&key
    ((:direction __direction) 0)
    ((:x __x) 0)
    ((:y __y) 0)
    )
   (send-super :init)
   (setq _direction (round __direction))
   (setq _x (round __x))
   (setq _y (round __y))
   self)
  (:direction
   (&optional __direction)
   (if __direction (setq _direction __direction)) _direction)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:serialization-length
   ()
   (+
    ;; uint16 _direction
    2
    ;; uint16 _x
    2
    ;; uint16 _y
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16 _direction
       (write-word _direction s)
     ;; uint16 _x
       (write-word _x s)
     ;; uint16 _y
       (write-word _y s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16 _direction
     (setq _direction (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _x
     (setq _x (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _y
     (setq _y (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get fov::FishState :md5sum-) "113921524c1e6a5b46daec728ccb3e30")
(setf (get fov::FishState :datatype-) "fov/FishState")
(setf (get fov::FishState :definition-)
      "# FishState.msg
uint16 direction
uint16 x
uint16 y
")



(provide :fov/FishState "113921524c1e6a5b46daec728ccb3e30")


