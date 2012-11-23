; Auto-generated. Do not edit!


(cl:in-package ar_recog-srv)


;//! \htmlinclude CalibrateDistance-request.msg.html

(cl:defclass <CalibrateDistance-request> (roslisp-msg-protocol:ros-message)
  ((dist
    :reader dist
    :initarg :dist
    :type cl:integer
    :initform 0))
)

(cl:defclass CalibrateDistance-request (<CalibrateDistance-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CalibrateDistance-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CalibrateDistance-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ar_recog-srv:<CalibrateDistance-request> is deprecated: use ar_recog-srv:CalibrateDistance-request instead.")))

(cl:ensure-generic-function 'dist-val :lambda-list '(m))
(cl:defmethod dist-val ((m <CalibrateDistance-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ar_recog-srv:dist-val is deprecated.  Use ar_recog-srv:dist instead.")
  (dist m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CalibrateDistance-request>) ostream)
  "Serializes a message object of type '<CalibrateDistance-request>"
  (cl:let* ((signed (cl:slot-value msg 'dist)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CalibrateDistance-request>) istream)
  "Deserializes a message object of type '<CalibrateDistance-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'dist) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CalibrateDistance-request>)))
  "Returns string type for a service object of type '<CalibrateDistance-request>"
  "ar_recog/CalibrateDistanceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CalibrateDistance-request)))
  "Returns string type for a service object of type 'CalibrateDistance-request"
  "ar_recog/CalibrateDistanceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CalibrateDistance-request>)))
  "Returns md5sum for a message object of type '<CalibrateDistance-request>"
  "b825ddadc18a94e1aff9d418f4e6cfee")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CalibrateDistance-request)))
  "Returns md5sum for a message object of type 'CalibrateDistance-request"
  "b825ddadc18a94e1aff9d418f4e6cfee")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CalibrateDistance-request>)))
  "Returns full string definition for message of type '<CalibrateDistance-request>"
  (cl:format cl:nil "int32 dist~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CalibrateDistance-request)))
  "Returns full string definition for message of type 'CalibrateDistance-request"
  (cl:format cl:nil "int32 dist~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CalibrateDistance-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CalibrateDistance-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CalibrateDistance-request
    (cl:cons ':dist (dist msg))
))
;//! \htmlinclude CalibrateDistance-response.msg.html

(cl:defclass <CalibrateDistance-response> (roslisp-msg-protocol:ros-message)
  ((aov
    :reader aov
    :initarg :aov
    :type cl:float
    :initform 0.0))
)

(cl:defclass CalibrateDistance-response (<CalibrateDistance-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CalibrateDistance-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CalibrateDistance-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ar_recog-srv:<CalibrateDistance-response> is deprecated: use ar_recog-srv:CalibrateDistance-response instead.")))

(cl:ensure-generic-function 'aov-val :lambda-list '(m))
(cl:defmethod aov-val ((m <CalibrateDistance-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ar_recog-srv:aov-val is deprecated.  Use ar_recog-srv:aov instead.")
  (aov m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CalibrateDistance-response>) ostream)
  "Serializes a message object of type '<CalibrateDistance-response>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'aov))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CalibrateDistance-response>) istream)
  "Deserializes a message object of type '<CalibrateDistance-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'aov) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CalibrateDistance-response>)))
  "Returns string type for a service object of type '<CalibrateDistance-response>"
  "ar_recog/CalibrateDistanceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CalibrateDistance-response)))
  "Returns string type for a service object of type 'CalibrateDistance-response"
  "ar_recog/CalibrateDistanceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CalibrateDistance-response>)))
  "Returns md5sum for a message object of type '<CalibrateDistance-response>"
  "b825ddadc18a94e1aff9d418f4e6cfee")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CalibrateDistance-response)))
  "Returns md5sum for a message object of type 'CalibrateDistance-response"
  "b825ddadc18a94e1aff9d418f4e6cfee")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CalibrateDistance-response>)))
  "Returns full string definition for message of type '<CalibrateDistance-response>"
  (cl:format cl:nil "float64 aov~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CalibrateDistance-response)))
  "Returns full string definition for message of type 'CalibrateDistance-response"
  (cl:format cl:nil "float64 aov~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CalibrateDistance-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CalibrateDistance-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CalibrateDistance-response
    (cl:cons ':aov (aov msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CalibrateDistance)))
  'CalibrateDistance-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CalibrateDistance)))
  'CalibrateDistance-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CalibrateDistance)))
  "Returns string type for a service object of type '<CalibrateDistance>"
  "ar_recog/CalibrateDistance")