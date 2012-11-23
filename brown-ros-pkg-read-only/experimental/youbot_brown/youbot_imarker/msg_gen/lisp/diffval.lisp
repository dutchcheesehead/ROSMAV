; Auto-generated. Do not edit!


(cl:in-package youbot_imarker-msg)


;//! \htmlinclude diffval.msg.html

(cl:defclass <diffval> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (xx
    :reader xx
    :initarg :xx
    :type cl:float
    :initform 0.0)
   (yy
    :reader yy
    :initarg :yy
    :type cl:float
    :initform 0.0)
   (zz
    :reader zz
    :initarg :zz
    :type cl:float
    :initform 0.0))
)

(cl:defclass diffval (<diffval>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <diffval>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'diffval)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name youbot_imarker-msg:<diffval> is deprecated: use youbot_imarker-msg:diffval instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <diffval>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader youbot_imarker-msg:header-val is deprecated.  Use youbot_imarker-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'xx-val :lambda-list '(m))
(cl:defmethod xx-val ((m <diffval>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader youbot_imarker-msg:xx-val is deprecated.  Use youbot_imarker-msg:xx instead.")
  (xx m))

(cl:ensure-generic-function 'yy-val :lambda-list '(m))
(cl:defmethod yy-val ((m <diffval>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader youbot_imarker-msg:yy-val is deprecated.  Use youbot_imarker-msg:yy instead.")
  (yy m))

(cl:ensure-generic-function 'zz-val :lambda-list '(m))
(cl:defmethod zz-val ((m <diffval>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader youbot_imarker-msg:zz-val is deprecated.  Use youbot_imarker-msg:zz instead.")
  (zz m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <diffval>) ostream)
  "Serializes a message object of type '<diffval>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'xx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'zz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <diffval>) istream)
  "Deserializes a message object of type '<diffval>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'xx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yy) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'zz) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<diffval>)))
  "Returns string type for a message object of type '<diffval>"
  "youbot_imarker/diffval")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'diffval)))
  "Returns string type for a message object of type 'diffval"
  "youbot_imarker/diffval")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<diffval>)))
  "Returns md5sum for a message object of type '<diffval>"
  "78a5bd8901d78d315b27b8ce4f334934")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'diffval)))
  "Returns md5sum for a message object of type 'diffval"
  "78a5bd8901d78d315b27b8ce4f334934")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<diffval>)))
  "Returns full string definition for message of type '<diffval>"
  (cl:format cl:nil "Header header~%~%float32 xx~%float32 yy~%float32 zz~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'diffval)))
  "Returns full string definition for message of type 'diffval"
  (cl:format cl:nil "Header header~%~%float32 xx~%float32 yy~%float32 zz~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <diffval>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <diffval>))
  "Converts a ROS message object to a list"
  (cl:list 'diffval
    (cl:cons ':header (header msg))
    (cl:cons ':xx (xx msg))
    (cl:cons ':yy (yy msg))
    (cl:cons ':zz (zz msg))
))
