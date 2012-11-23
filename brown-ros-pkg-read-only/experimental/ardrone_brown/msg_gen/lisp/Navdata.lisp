; Auto-generated. Do not edit!


(cl:in-package ardrone_brown-msg)


;//! \htmlinclude Navdata.msg.html

(cl:defclass <Navdata> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (batteryPercent
    :reader batteryPercent
    :initarg :batteryPercent
    :type cl:float
    :initform 0.0)
   (rotX
    :reader rotX
    :initarg :rotX
    :type cl:float
    :initform 0.0)
   (rotY
    :reader rotY
    :initarg :rotY
    :type cl:float
    :initform 0.0)
   (rotZ
    :reader rotZ
    :initarg :rotZ
    :type cl:float
    :initform 0.0)
   (altd
    :reader altd
    :initarg :altd
    :type cl:integer
    :initform 0)
   (vx
    :reader vx
    :initarg :vx
    :type cl:float
    :initform 0.0)
   (vy
    :reader vy
    :initarg :vy
    :type cl:float
    :initform 0.0)
   (vz
    :reader vz
    :initarg :vz
    :type cl:float
    :initform 0.0)
   (tm
    :reader tm
    :initarg :tm
    :type cl:float
    :initform 0.0))
)

(cl:defclass Navdata (<Navdata>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Navdata>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Navdata)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ardrone_brown-msg:<Navdata> is deprecated: use ardrone_brown-msg:Navdata instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Navdata>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ardrone_brown-msg:header-val is deprecated.  Use ardrone_brown-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'batteryPercent-val :lambda-list '(m))
(cl:defmethod batteryPercent-val ((m <Navdata>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ardrone_brown-msg:batteryPercent-val is deprecated.  Use ardrone_brown-msg:batteryPercent instead.")
  (batteryPercent m))

(cl:ensure-generic-function 'rotX-val :lambda-list '(m))
(cl:defmethod rotX-val ((m <Navdata>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ardrone_brown-msg:rotX-val is deprecated.  Use ardrone_brown-msg:rotX instead.")
  (rotX m))

(cl:ensure-generic-function 'rotY-val :lambda-list '(m))
(cl:defmethod rotY-val ((m <Navdata>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ardrone_brown-msg:rotY-val is deprecated.  Use ardrone_brown-msg:rotY instead.")
  (rotY m))

(cl:ensure-generic-function 'rotZ-val :lambda-list '(m))
(cl:defmethod rotZ-val ((m <Navdata>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ardrone_brown-msg:rotZ-val is deprecated.  Use ardrone_brown-msg:rotZ instead.")
  (rotZ m))

(cl:ensure-generic-function 'altd-val :lambda-list '(m))
(cl:defmethod altd-val ((m <Navdata>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ardrone_brown-msg:altd-val is deprecated.  Use ardrone_brown-msg:altd instead.")
  (altd m))

(cl:ensure-generic-function 'vx-val :lambda-list '(m))
(cl:defmethod vx-val ((m <Navdata>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ardrone_brown-msg:vx-val is deprecated.  Use ardrone_brown-msg:vx instead.")
  (vx m))

(cl:ensure-generic-function 'vy-val :lambda-list '(m))
(cl:defmethod vy-val ((m <Navdata>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ardrone_brown-msg:vy-val is deprecated.  Use ardrone_brown-msg:vy instead.")
  (vy m))

(cl:ensure-generic-function 'vz-val :lambda-list '(m))
(cl:defmethod vz-val ((m <Navdata>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ardrone_brown-msg:vz-val is deprecated.  Use ardrone_brown-msg:vz instead.")
  (vz m))

(cl:ensure-generic-function 'tm-val :lambda-list '(m))
(cl:defmethod tm-val ((m <Navdata>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ardrone_brown-msg:tm-val is deprecated.  Use ardrone_brown-msg:tm instead.")
  (tm m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Navdata>) ostream)
  "Serializes a message object of type '<Navdata>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'batteryPercent))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rotX))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rotY))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rotZ))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'altd)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tm))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Navdata>) istream)
  "Deserializes a message object of type '<Navdata>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'batteryPercent) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rotX) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rotY) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rotZ) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'altd) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vy) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vz) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tm) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Navdata>)))
  "Returns string type for a message object of type '<Navdata>"
  "ardrone_brown/Navdata")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Navdata)))
  "Returns string type for a message object of type 'Navdata"
  "ardrone_brown/Navdata")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Navdata>)))
  "Returns md5sum for a message object of type '<Navdata>"
  "0e8fc2a4b7f377e10e22371c42bfd78b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Navdata)))
  "Returns md5sum for a message object of type 'Navdata"
  "0e8fc2a4b7f377e10e22371c42bfd78b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Navdata>)))
  "Returns full string definition for message of type '<Navdata>"
  (cl:format cl:nil "Header header~%~%# 0 means no battery, 100 means full battery~%float32 batteryPercent~%~%# left/right tilt in degrees (rotation about the X axis)~%float32 rotX~%~%# forward/backward tilt in degrees (rotation about the Y axis)~%float32 rotY~%~%# orientation in degrees (rotation about the Z axis)~%float32 rotZ~%~%# estimated altitude (cm)~%int32 altd~%~%# linear velocity (mm/sec)~%float32 vx~%~%# linear velocity (mm/sec)~%float32 vy~%~%# linear velocity (mm/sec)~%float32 vz~%~%#time stamp~%float32 tm~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Navdata)))
  "Returns full string definition for message of type 'Navdata"
  (cl:format cl:nil "Header header~%~%# 0 means no battery, 100 means full battery~%float32 batteryPercent~%~%# left/right tilt in degrees (rotation about the X axis)~%float32 rotX~%~%# forward/backward tilt in degrees (rotation about the Y axis)~%float32 rotY~%~%# orientation in degrees (rotation about the Z axis)~%float32 rotZ~%~%# estimated altitude (cm)~%int32 altd~%~%# linear velocity (mm/sec)~%float32 vx~%~%# linear velocity (mm/sec)~%float32 vy~%~%# linear velocity (mm/sec)~%float32 vz~%~%#time stamp~%float32 tm~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Navdata>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Navdata>))
  "Converts a ROS message object to a list"
  (cl:list 'Navdata
    (cl:cons ':header (header msg))
    (cl:cons ':batteryPercent (batteryPercent msg))
    (cl:cons ':rotX (rotX msg))
    (cl:cons ':rotY (rotY msg))
    (cl:cons ':rotZ (rotZ msg))
    (cl:cons ':altd (altd msg))
    (cl:cons ':vx (vx msg))
    (cl:cons ':vy (vy msg))
    (cl:cons ':vz (vz msg))
    (cl:cons ':tm (tm msg))
))
