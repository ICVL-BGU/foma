;; Auto-generated. Do not edit!


(when (boundp 'fov::Light)
  (if (not (find-package "FOV"))
    (make-package "FOV"))
  (shadow 'Light (find-package "FOV")))
(unless (find-package "FOV::LIGHT")
  (make-package "FOV::LIGHT"))
(unless (find-package "FOV::LIGHTREQUEST")
  (make-package "FOV::LIGHTREQUEST"))
(unless (find-package "FOV::LIGHTRESPONSE")
  (make-package "FOV::LIGHTRESPONSE"))

(in-package "ROS")





(defclass fov::LightRequest
  :super ros::object
  :slots (_data ))

(defmethod fov::LightRequest
  (:init
   (&key
    ((:data __data) 0.0)
    )
   (send-super :init)
   (setq _data (float __data))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; float32 _data
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _data
       (sys::poke _data (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _data
     (setq _data (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass fov::LightResponse
  :super ros::object
  :slots (_result ))

(defmethod fov::LightResponse
  (:init
   (&key
    ((:result __result) nil)
    )
   (send-super :init)
   (setq _result __result)
   self)
  (:result
   (&optional (__result :null))
   (if (not (eq __result :null)) (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; bool _result
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _result
       (if _result (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _result
     (setq _result (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass fov::Light
  :super ros::object
  :slots ())

(setf (get fov::Light :md5sum-) "b21ecd0878e18358dcca817a8b8fc556")
(setf (get fov::Light :datatype-) "fov/Light")
(setf (get fov::Light :request) fov::LightRequest)
(setf (get fov::Light :response) fov::LightResponse)

(defmethod fov::LightRequest
  (:response () (instance fov::LightResponse :init)))

(setf (get fov::LightRequest :md5sum-) "b21ecd0878e18358dcca817a8b8fc556")
(setf (get fov::LightRequest :datatype-) "fov/LightRequest")
(setf (get fov::LightRequest :definition-)
      "float32 data
---
bool result
")

(setf (get fov::LightResponse :md5sum-) "b21ecd0878e18358dcca817a8b8fc556")
(setf (get fov::LightResponse :datatype-) "fov/LightResponse")
(setf (get fov::LightResponse :definition-)
      "float32 data
---
bool result
")



(provide :fov/Light "b21ecd0878e18358dcca817a8b8fc556")


