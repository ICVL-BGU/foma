; Auto-generated. Do not edit!


(cl:in-package fov-msg)


;//! \htmlinclude FomaLocation.msg.html

(cl:defclass <FomaLocation> (roslisp-msg-protocol:ros-message)
  ((image
    :reader image
    :initarg :image
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (world
    :reader world
    :initarg :world
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass FomaLocation (<FomaLocation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FomaLocation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FomaLocation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fov-msg:<FomaLocation> is deprecated: use fov-msg:FomaLocation instead.")))

(cl:ensure-generic-function 'image-val :lambda-list '(m))
(cl:defmethod image-val ((m <FomaLocation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fov-msg:image-val is deprecated.  Use fov-msg:image instead.")
  (image m))

(cl:ensure-generic-function 'world-val :lambda-list '(m))
(cl:defmethod world-val ((m <FomaLocation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fov-msg:world-val is deprecated.  Use fov-msg:world instead.")
  (world m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FomaLocation>) ostream)
  "Serializes a message object of type '<FomaLocation>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'image) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'world) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FomaLocation>) istream)
  "Deserializes a message object of type '<FomaLocation>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'image) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'world) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FomaLocation>)))
  "Returns string type for a message object of type '<FomaLocation>"
  "fov/FomaLocation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FomaLocation)))
  "Returns string type for a message object of type 'FomaLocation"
  "fov/FomaLocation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FomaLocation>)))
  "Returns md5sum for a message object of type '<FomaLocation>"
  "5b07c96efdecb7282bf61c35bd56a0d2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FomaLocation)))
  "Returns md5sum for a message object of type 'FomaLocation"
  "5b07c96efdecb7282bf61c35bd56a0d2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FomaLocation>)))
  "Returns full string definition for message of type '<FomaLocation>"
  (cl:format cl:nil "geometry_msgs/Point image~%geometry_msgs/Point world~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FomaLocation)))
  "Returns full string definition for message of type 'FomaLocation"
  (cl:format cl:nil "geometry_msgs/Point image~%geometry_msgs/Point world~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FomaLocation>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'image))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'world))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FomaLocation>))
  "Converts a ROS message object to a list"
  (cl:list 'FomaLocation
    (cl:cons ':image (image msg))
    (cl:cons ':world (world msg))
))
