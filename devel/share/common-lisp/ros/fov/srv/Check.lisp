; Auto-generated. Do not edit!


(cl:in-package fov-srv)


;//! \htmlinclude Check-request.msg.html

(cl:defclass <Check-request> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Check-request (<Check-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Check-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Check-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fov-srv:<Check-request> is deprecated: use fov-srv:Check-request instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <Check-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fov-srv:data-val is deprecated.  Use fov-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Check-request>) ostream)
  "Serializes a message object of type '<Check-request>"
  (cl:let* ((signed (cl:slot-value msg 'data)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Check-request>) istream)
  "Deserializes a message object of type '<Check-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'data) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Check-request>)))
  "Returns string type for a service object of type '<Check-request>"
  "fov/CheckRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Check-request)))
  "Returns string type for a service object of type 'Check-request"
  "fov/CheckRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Check-request>)))
  "Returns md5sum for a message object of type '<Check-request>"
  "f8c2075a484d0d4e2762c28ac0073c54")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Check-request)))
  "Returns md5sum for a message object of type 'Check-request"
  "f8c2075a484d0d4e2762c28ac0073c54")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Check-request>)))
  "Returns full string definition for message of type '<Check-request>"
  (cl:format cl:nil "int8 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Check-request)))
  "Returns full string definition for message of type 'Check-request"
  (cl:format cl:nil "int8 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Check-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Check-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Check-request
    (cl:cons ':data (data msg))
))
;//! \htmlinclude Check-response.msg.html

(cl:defclass <Check-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Check-response (<Check-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Check-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Check-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fov-srv:<Check-response> is deprecated: use fov-srv:Check-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <Check-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fov-srv:result-val is deprecated.  Use fov-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Check-response>) ostream)
  "Serializes a message object of type '<Check-response>"
  (cl:let* ((signed (cl:slot-value msg 'result)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Check-response>) istream)
  "Deserializes a message object of type '<Check-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Check-response>)))
  "Returns string type for a service object of type '<Check-response>"
  "fov/CheckResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Check-response)))
  "Returns string type for a service object of type 'Check-response"
  "fov/CheckResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Check-response>)))
  "Returns md5sum for a message object of type '<Check-response>"
  "f8c2075a484d0d4e2762c28ac0073c54")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Check-response)))
  "Returns md5sum for a message object of type 'Check-response"
  "f8c2075a484d0d4e2762c28ac0073c54")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Check-response>)))
  "Returns full string definition for message of type '<Check-response>"
  (cl:format cl:nil "int8 result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Check-response)))
  "Returns full string definition for message of type 'Check-response"
  (cl:format cl:nil "int8 result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Check-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Check-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Check-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Check)))
  'Check-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Check)))
  'Check-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Check)))
  "Returns string type for a service object of type '<Check>"
  "fov/Check")