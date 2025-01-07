;; Auto-generated. Do not edit!


(when (boundp 'fov::Check)
  (if (not (find-package "FOV"))
    (make-package "FOV"))
  (shadow 'Check (find-package "FOV")))
(unless (find-package "FOV::CHECK")
  (make-package "FOV::CHECK"))
(unless (find-package "FOV::CHECKREQUEST")
  (make-package "FOV::CHECKREQUEST"))
(unless (find-package "FOV::CHECKRESPONSE")
  (make-package "FOV::CHECKRESPONSE"))

(in-package "ROS")





(defclass fov::CheckRequest
  :super ros::object
  :slots (_data ))

(defmethod fov::CheckRequest
  (:init
   (&key
    ((:data __data) 0)
    )
   (send-super :init)
   (setq _data (round __data))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; int8 _data
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _data
       (write-byte _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _data
     (setq _data (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _data 127) (setq _data (- _data 256)))
   ;;
   self)
  )

(defclass fov::CheckResponse
  :super ros::object
  :slots (_result ))

(defmethod fov::CheckResponse
  (:init
   (&key
    ((:result __result) 0)
    )
   (send-super :init)
   (setq _result (round __result))
   self)
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; int8 _result
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _result
       (write-byte _result s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _result
     (setq _result (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _result 127) (setq _result (- _result 256)))
   ;;
   self)
  )

(defclass fov::Check
  :super ros::object
  :slots ())

(setf (get fov::Check :md5sum-) "f8c2075a484d0d4e2762c28ac0073c54")
(setf (get fov::Check :datatype-) "fov/Check")
(setf (get fov::Check :request) fov::CheckRequest)
(setf (get fov::Check :response) fov::CheckResponse)

(defmethod fov::CheckRequest
  (:response () (instance fov::CheckResponse :init)))

(setf (get fov::CheckRequest :md5sum-) "f8c2075a484d0d4e2762c28ac0073c54")
(setf (get fov::CheckRequest :datatype-) "fov/CheckRequest")
(setf (get fov::CheckRequest :definition-)
      "int8 data
---
int8 result
")

(setf (get fov::CheckResponse :md5sum-) "f8c2075a484d0d4e2762c28ac0073c54")
(setf (get fov::CheckResponse :datatype-) "fov/CheckResponse")
(setf (get fov::CheckResponse :definition-)
      "int8 data
---
int8 result
")



(provide :fov/Check "f8c2075a484d0d4e2762c28ac0073c54")


