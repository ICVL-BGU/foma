; Auto-generated. Do not edit!


(cl:in-package fov-msg)


;//! \htmlinclude FishState.msg.html

(cl:defclass <FishState> (roslisp-msg-protocol:ros-message)
  ((direction
    :reader direction
    :initarg :direction
    :type cl:fixnum
    :initform 0)
   (x
    :reader x
    :initarg :x
    :type cl:fixnum
    :initform 0)
   (y
    :reader y
    :initarg :y
    :type cl:fixnum
    :initform 0))
)

(cl:defclass FishState (<FishState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FishState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FishState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fov-msg:<FishState> is deprecated: use fov-msg:FishState instead.")))

(cl:ensure-generic-function 'direction-val :lambda-list '(m))
(cl:defmethod direction-val ((m <FishState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fov-msg:direction-val is deprecated.  Use fov-msg:direction instead.")
  (direction m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <FishState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fov-msg:x-val is deprecated.  Use fov-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <FishState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fov-msg:y-val is deprecated.  Use fov-msg:y instead.")
  (y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FishState>) ostream)
  "Serializes a message object of type '<FishState>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'direction)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'direction)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'y)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'y)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FishState>) istream)
  "Deserializes a message object of type '<FishState>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'direction)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'direction)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'y)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'y)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FishState>)))
  "Returns string type for a message object of type '<FishState>"
  "fov/FishState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FishState)))
  "Returns string type for a message object of type 'FishState"
  "fov/FishState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FishState>)))
  "Returns md5sum for a message object of type '<FishState>"
  "113921524c1e6a5b46daec728ccb3e30")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FishState)))
  "Returns md5sum for a message object of type 'FishState"
  "113921524c1e6a5b46daec728ccb3e30")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FishState>)))
  "Returns full string definition for message of type '<FishState>"
  (cl:format cl:nil "# FishState.msg~%uint16 direction~%uint16 x~%uint16 y~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FishState)))
  "Returns full string definition for message of type 'FishState"
  (cl:format cl:nil "# FishState.msg~%uint16 direction~%uint16 x~%uint16 y~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FishState>))
  (cl:+ 0
     2
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FishState>))
  "Converts a ROS message object to a list"
  (cl:list 'FishState
    (cl:cons ':direction (direction msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
))
