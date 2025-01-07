; Auto-generated. Do not edit!


(cl:in-package fov-srv)


;//! \htmlinclude Light-request.msg.html

(cl:defclass <Light-request> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:float
    :initform 0.0))
)

(cl:defclass Light-request (<Light-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Light-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Light-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fov-srv:<Light-request> is deprecated: use fov-srv:Light-request instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <Light-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fov-srv:data-val is deprecated.  Use fov-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Light-request>) ostream)
  "Serializes a message object of type '<Light-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Light-request>) istream)
  "Deserializes a message object of type '<Light-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'data) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Light-request>)))
  "Returns string type for a service object of type '<Light-request>"
  "fov/LightRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Light-request)))
  "Returns string type for a service object of type 'Light-request"
  "fov/LightRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Light-request>)))
  "Returns md5sum for a message object of type '<Light-request>"
  "b21ecd0878e18358dcca817a8b8fc556")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Light-request)))
  "Returns md5sum for a message object of type 'Light-request"
  "b21ecd0878e18358dcca817a8b8fc556")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Light-request>)))
  "Returns full string definition for message of type '<Light-request>"
  (cl:format cl:nil "float32 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Light-request)))
  "Returns full string definition for message of type 'Light-request"
  (cl:format cl:nil "float32 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Light-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Light-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Light-request
    (cl:cons ':data (data msg))
))
;//! \htmlinclude Light-response.msg.html

(cl:defclass <Light-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Light-response (<Light-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Light-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Light-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fov-srv:<Light-response> is deprecated: use fov-srv:Light-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <Light-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fov-srv:result-val is deprecated.  Use fov-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Light-response>) ostream)
  "Serializes a message object of type '<Light-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'result) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Light-response>) istream)
  "Deserializes a message object of type '<Light-response>"
    (cl:setf (cl:slot-value msg 'result) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Light-response>)))
  "Returns string type for a service object of type '<Light-response>"
  "fov/LightResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Light-response)))
  "Returns string type for a service object of type 'Light-response"
  "fov/LightResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Light-response>)))
  "Returns md5sum for a message object of type '<Light-response>"
  "b21ecd0878e18358dcca817a8b8fc556")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Light-response)))
  "Returns md5sum for a message object of type 'Light-response"
  "b21ecd0878e18358dcca817a8b8fc556")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Light-response>)))
  "Returns full string definition for message of type '<Light-response>"
  (cl:format cl:nil "bool result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Light-response)))
  "Returns full string definition for message of type 'Light-response"
  (cl:format cl:nil "bool result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Light-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Light-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Light-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Light)))
  'Light-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Light)))
  'Light-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Light)))
  "Returns string type for a service object of type '<Light>"
  "fov/Light")