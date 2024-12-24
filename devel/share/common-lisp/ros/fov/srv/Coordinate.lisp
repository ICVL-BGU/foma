; Auto-generated. Do not edit!


(cl:in-package fov-srv)


;//! \htmlinclude Coordinate-request.msg.html

(cl:defclass <Coordinate-request> (roslisp-msg-protocol:ros-message)
  ((idx
    :reader idx
    :initarg :idx
    :type cl:fixnum
    :initform 0)
   (xmin
    :reader xmin
    :initarg :xmin
    :type cl:integer
    :initform 0)
   (xmax
    :reader xmax
    :initarg :xmax
    :type cl:integer
    :initform 0)
   (ymin
    :reader ymin
    :initarg :ymin
    :type cl:integer
    :initform 0)
   (ymax
    :reader ymax
    :initarg :ymax
    :type cl:integer
    :initform 0)
   (mask
    :reader mask
    :initarg :mask
    :type std_msgs-msg:UInt8MultiArray
    :initform (cl:make-instance 'std_msgs-msg:UInt8MultiArray)))
)

(cl:defclass Coordinate-request (<Coordinate-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Coordinate-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Coordinate-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fov-srv:<Coordinate-request> is deprecated: use fov-srv:Coordinate-request instead.")))

(cl:ensure-generic-function 'idx-val :lambda-list '(m))
(cl:defmethod idx-val ((m <Coordinate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fov-srv:idx-val is deprecated.  Use fov-srv:idx instead.")
  (idx m))

(cl:ensure-generic-function 'xmin-val :lambda-list '(m))
(cl:defmethod xmin-val ((m <Coordinate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fov-srv:xmin-val is deprecated.  Use fov-srv:xmin instead.")
  (xmin m))

(cl:ensure-generic-function 'xmax-val :lambda-list '(m))
(cl:defmethod xmax-val ((m <Coordinate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fov-srv:xmax-val is deprecated.  Use fov-srv:xmax instead.")
  (xmax m))

(cl:ensure-generic-function 'ymin-val :lambda-list '(m))
(cl:defmethod ymin-val ((m <Coordinate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fov-srv:ymin-val is deprecated.  Use fov-srv:ymin instead.")
  (ymin m))

(cl:ensure-generic-function 'ymax-val :lambda-list '(m))
(cl:defmethod ymax-val ((m <Coordinate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fov-srv:ymax-val is deprecated.  Use fov-srv:ymax instead.")
  (ymax m))

(cl:ensure-generic-function 'mask-val :lambda-list '(m))
(cl:defmethod mask-val ((m <Coordinate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fov-srv:mask-val is deprecated.  Use fov-srv:mask instead.")
  (mask m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Coordinate-request>) ostream)
  "Serializes a message object of type '<Coordinate-request>"
  (cl:let* ((signed (cl:slot-value msg 'idx)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'xmin)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'xmax)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'ymin)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'ymax)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'mask) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Coordinate-request>) istream)
  "Deserializes a message object of type '<Coordinate-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'idx) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'xmin) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'xmax) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ymin) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ymax) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'mask) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Coordinate-request>)))
  "Returns string type for a service object of type '<Coordinate-request>"
  "fov/CoordinateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Coordinate-request)))
  "Returns string type for a service object of type 'Coordinate-request"
  "fov/CoordinateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Coordinate-request>)))
  "Returns md5sum for a message object of type '<Coordinate-request>"
  "59330ecd12d514a4552b7f072136e8ea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Coordinate-request)))
  "Returns md5sum for a message object of type 'Coordinate-request"
  "59330ecd12d514a4552b7f072136e8ea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Coordinate-request>)))
  "Returns full string definition for message of type '<Coordinate-request>"
  (cl:format cl:nil "int8 idx~%int32 xmin~%int32 xmax~%int32 ymin~%int32 ymax~%std_msgs/UInt8MultiArray mask~%~%================================================================================~%MSG: std_msgs/UInt8MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%uint8[]           data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Coordinate-request)))
  "Returns full string definition for message of type 'Coordinate-request"
  (cl:format cl:nil "int8 idx~%int32 xmin~%int32 xmax~%int32 ymin~%int32 ymax~%std_msgs/UInt8MultiArray mask~%~%================================================================================~%MSG: std_msgs/UInt8MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%uint8[]           data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Coordinate-request>))
  (cl:+ 0
     1
     4
     4
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'mask))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Coordinate-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Coordinate-request
    (cl:cons ':idx (idx msg))
    (cl:cons ':xmin (xmin msg))
    (cl:cons ':xmax (xmax msg))
    (cl:cons ':ymin (ymin msg))
    (cl:cons ':ymax (ymax msg))
    (cl:cons ':mask (mask msg))
))
;//! \htmlinclude Coordinate-response.msg.html

(cl:defclass <Coordinate-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Coordinate-response (<Coordinate-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Coordinate-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Coordinate-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fov-srv:<Coordinate-response> is deprecated: use fov-srv:Coordinate-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Coordinate-response>) ostream)
  "Serializes a message object of type '<Coordinate-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Coordinate-response>) istream)
  "Deserializes a message object of type '<Coordinate-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Coordinate-response>)))
  "Returns string type for a service object of type '<Coordinate-response>"
  "fov/CoordinateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Coordinate-response)))
  "Returns string type for a service object of type 'Coordinate-response"
  "fov/CoordinateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Coordinate-response>)))
  "Returns md5sum for a message object of type '<Coordinate-response>"
  "59330ecd12d514a4552b7f072136e8ea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Coordinate-response)))
  "Returns md5sum for a message object of type 'Coordinate-response"
  "59330ecd12d514a4552b7f072136e8ea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Coordinate-response>)))
  "Returns full string definition for message of type '<Coordinate-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Coordinate-response)))
  "Returns full string definition for message of type 'Coordinate-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Coordinate-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Coordinate-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Coordinate-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Coordinate)))
  'Coordinate-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Coordinate)))
  'Coordinate-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Coordinate)))
  "Returns string type for a service object of type '<Coordinate>"
  "fov/Coordinate")