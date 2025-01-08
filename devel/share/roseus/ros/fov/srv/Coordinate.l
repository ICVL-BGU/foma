;; Auto-generated. Do not edit!


(when (boundp 'fov::Coordinate)
  (if (not (find-package "FOV"))
    (make-package "FOV"))
  (shadow 'Coordinate (find-package "FOV")))
(unless (find-package "FOV::COORDINATE")
  (make-package "FOV::COORDINATE"))
(unless (find-package "FOV::COORDINATEREQUEST")
  (make-package "FOV::COORDINATEREQUEST"))
(unless (find-package "FOV::COORDINATERESPONSE")
  (make-package "FOV::COORDINATERESPONSE"))

(in-package "ROS")

(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))




(defclass fov::CoordinateRequest
  :super ros::object
  :slots (_idx _xmin _xmax _ymin _ymax _mask ))

(defmethod fov::CoordinateRequest
  (:init
   (&key
    ((:idx __idx) 0)
    ((:xmin __xmin) 0)
    ((:xmax __xmax) 0)
    ((:ymin __ymin) 0)
    ((:ymax __ymax) 0)
    ((:mask __mask) (instance std_msgs::UInt8MultiArray :init))
    )
   (send-super :init)
   (setq _idx (round __idx))
   (setq _xmin (round __xmin))
   (setq _xmax (round __xmax))
   (setq _ymin (round __ymin))
   (setq _ymax (round __ymax))
   (setq _mask __mask)
   self)
  (:idx
   (&optional __idx)
   (if __idx (setq _idx __idx)) _idx)
  (:xmin
   (&optional __xmin)
   (if __xmin (setq _xmin __xmin)) _xmin)
  (:xmax
   (&optional __xmax)
   (if __xmax (setq _xmax __xmax)) _xmax)
  (:ymin
   (&optional __ymin)
   (if __ymin (setq _ymin __ymin)) _ymin)
  (:ymax
   (&optional __ymax)
   (if __ymax (setq _ymax __ymax)) _ymax)
  (:mask
   (&rest __mask)
   (if (keywordp (car __mask))
       (send* _mask __mask)
     (progn
       (if __mask (setq _mask (car __mask)))
       _mask)))
  (:serialization-length
   ()
   (+
    ;; int8 _idx
    1
    ;; int32 _xmin
    4
    ;; int32 _xmax
    4
    ;; int32 _ymin
    4
    ;; int32 _ymax
    4
    ;; std_msgs/UInt8MultiArray _mask
    (send _mask :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _idx
       (write-byte _idx s)
     ;; int32 _xmin
       (write-long _xmin s)
     ;; int32 _xmax
       (write-long _xmax s)
     ;; int32 _ymin
       (write-long _ymin s)
     ;; int32 _ymax
       (write-long _ymax s)
     ;; std_msgs/UInt8MultiArray _mask
       (send _mask :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _idx
     (setq _idx (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _idx 127) (setq _idx (- _idx 256)))
   ;; int32 _xmin
     (setq _xmin (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _xmax
     (setq _xmax (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _ymin
     (setq _ymin (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _ymax
     (setq _ymax (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; std_msgs/UInt8MultiArray _mask
     (send _mask :deserialize buf ptr-) (incf ptr- (send _mask :serialization-length))
   ;;
   self)
  )

(defclass fov::CoordinateResponse
  :super ros::object
  :slots ())

(defmethod fov::CoordinateResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass fov::Coordinate
  :super ros::object
  :slots ())

(setf (get fov::Coordinate :md5sum-) "59330ecd12d514a4552b7f072136e8ea")
(setf (get fov::Coordinate :datatype-) "fov/Coordinate")
(setf (get fov::Coordinate :request) fov::CoordinateRequest)
(setf (get fov::Coordinate :response) fov::CoordinateResponse)

(defmethod fov::CoordinateRequest
  (:response () (instance fov::CoordinateResponse :init)))

(setf (get fov::CoordinateRequest :md5sum-) "59330ecd12d514a4552b7f072136e8ea")
(setf (get fov::CoordinateRequest :datatype-) "fov/CoordinateRequest")
(setf (get fov::CoordinateRequest :definition-)
      "int8 idx
int32 xmin
int32 xmax
int32 ymin
int32 ymax
std_msgs/UInt8MultiArray mask

================================================================================
MSG: std_msgs/UInt8MultiArray
# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
uint8[]           data          # array of data


================================================================================
MSG: std_msgs/MultiArrayLayout
# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
# 
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = \"height\"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = \"width\"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = \"channel\"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
---
")

(setf (get fov::CoordinateResponse :md5sum-) "59330ecd12d514a4552b7f072136e8ea")
(setf (get fov::CoordinateResponse :datatype-) "fov/CoordinateResponse")
(setf (get fov::CoordinateResponse :definition-)
      "int8 idx
int32 xmin
int32 xmax
int32 ymin
int32 ymax
std_msgs/UInt8MultiArray mask

================================================================================
MSG: std_msgs/UInt8MultiArray
# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
uint8[]           data          # array of data


================================================================================
MSG: std_msgs/MultiArrayLayout
# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
# 
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = \"height\"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = \"width\"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = \"channel\"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
---
")



(provide :fov/Coordinate "59330ecd12d514a4552b7f072136e8ea")


