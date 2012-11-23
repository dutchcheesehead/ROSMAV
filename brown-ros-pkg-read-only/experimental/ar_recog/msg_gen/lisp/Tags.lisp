; Auto-generated. Do not edit!


(cl:in-package ar_recog-msg)


;//! \htmlinclude Tags.msg.html

(cl:defclass <Tags> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (image_width
    :reader image_width
    :initarg :image_width
    :type cl:integer
    :initform 0)
   (image_height
    :reader image_height
    :initarg :image_height
    :type cl:integer
    :initform 0)
   (angle_of_view
    :reader angle_of_view
    :initarg :angle_of_view
    :type cl:float
    :initform 0.0)
   (tag_count
    :reader tag_count
    :initarg :tag_count
    :type cl:integer
    :initform 0)
   (tags
    :reader tags
    :initarg :tags
    :type (cl:vector ar_recog-msg:Tag)
   :initform (cl:make-array 0 :element-type 'ar_recog-msg:Tag :initial-element (cl:make-instance 'ar_recog-msg:Tag))))
)

(cl:defclass Tags (<Tags>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Tags>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Tags)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ar_recog-msg:<Tags> is deprecated: use ar_recog-msg:Tags instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Tags>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ar_recog-msg:header-val is deprecated.  Use ar_recog-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'image_width-val :lambda-list '(m))
(cl:defmethod image_width-val ((m <Tags>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ar_recog-msg:image_width-val is deprecated.  Use ar_recog-msg:image_width instead.")
  (image_width m))

(cl:ensure-generic-function 'image_height-val :lambda-list '(m))
(cl:defmethod image_height-val ((m <Tags>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ar_recog-msg:image_height-val is deprecated.  Use ar_recog-msg:image_height instead.")
  (image_height m))

(cl:ensure-generic-function 'angle_of_view-val :lambda-list '(m))
(cl:defmethod angle_of_view-val ((m <Tags>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ar_recog-msg:angle_of_view-val is deprecated.  Use ar_recog-msg:angle_of_view instead.")
  (angle_of_view m))

(cl:ensure-generic-function 'tag_count-val :lambda-list '(m))
(cl:defmethod tag_count-val ((m <Tags>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ar_recog-msg:tag_count-val is deprecated.  Use ar_recog-msg:tag_count instead.")
  (tag_count m))

(cl:ensure-generic-function 'tags-val :lambda-list '(m))
(cl:defmethod tags-val ((m <Tags>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ar_recog-msg:tags-val is deprecated.  Use ar_recog-msg:tags instead.")
  (tags m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Tags>) ostream)
  "Serializes a message object of type '<Tags>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'image_width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'image_width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'image_width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'image_width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'image_height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'image_height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'image_height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'image_height)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'angle_of_view))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tag_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tag_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'tag_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'tag_count)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'tags))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'tags))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Tags>) istream)
  "Deserializes a message object of type '<Tags>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'image_width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'image_width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'image_width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'image_width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'image_height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'image_height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'image_height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'image_height)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle_of_view) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tag_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tag_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'tag_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'tag_count)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'tags) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'tags)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'ar_recog-msg:Tag))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Tags>)))
  "Returns string type for a message object of type '<Tags>"
  "ar_recog/Tags")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Tags)))
  "Returns string type for a message object of type 'Tags"
  "ar_recog/Tags")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Tags>)))
  "Returns md5sum for a message object of type '<Tags>"
  "9fd9ccebe014b769dcb84bc4557ac837")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Tags)))
  "Returns md5sum for a message object of type 'Tags"
  "9fd9ccebe014b769dcb84bc4557ac837")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Tags>)))
  "Returns full string definition for message of type '<Tags>"
  (cl:format cl:nil "Header header~%uint32 image_width~%uint32 image_height~%float64 angle_of_view~%uint32 tag_count~%Tag[] tags~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ar_recog/Tag~%# All screen measurements are in pixels, all spatial measurements are in meters.~%# Angles in radians.~%uint32 id~%# This is a rating of confidence in the tag pattern identification 0 < cf < 1.~%float64 cf~%uint32 x~%uint32 y~%# The 'diameter' is the square root of the tag's actual area, as estimated by ~%# the AR software.  You can use it to check the cf confidence.~%float64 diameter~%# This is the estimated distance from viewer to the center of the tag.~%float64 distance~%float64 xRot ~%float64 yRot ~%float64 zRot~%float64 xMetric~%float64 yMetric~%float64 zMetric~%# Screen coordinates of the four corners.~%float64[8] cwCorners~%# FOR TESTING ONLY~%# Uncommenting this and uncommenting the similarly-marked lines in ~%# ar_recog.cpp will put the ARToolkit rotation matrix into the Tag message,~%# which can be useful for debugging and testing.~%# float64[3] c3~%# float64[3] t0~%# float64[3] t1~%# float64[3] t2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Tags)))
  "Returns full string definition for message of type 'Tags"
  (cl:format cl:nil "Header header~%uint32 image_width~%uint32 image_height~%float64 angle_of_view~%uint32 tag_count~%Tag[] tags~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ar_recog/Tag~%# All screen measurements are in pixels, all spatial measurements are in meters.~%# Angles in radians.~%uint32 id~%# This is a rating of confidence in the tag pattern identification 0 < cf < 1.~%float64 cf~%uint32 x~%uint32 y~%# The 'diameter' is the square root of the tag's actual area, as estimated by ~%# the AR software.  You can use it to check the cf confidence.~%float64 diameter~%# This is the estimated distance from viewer to the center of the tag.~%float64 distance~%float64 xRot ~%float64 yRot ~%float64 zRot~%float64 xMetric~%float64 yMetric~%float64 zMetric~%# Screen coordinates of the four corners.~%float64[8] cwCorners~%# FOR TESTING ONLY~%# Uncommenting this and uncommenting the similarly-marked lines in ~%# ar_recog.cpp will put the ARToolkit rotation matrix into the Tag message,~%# which can be useful for debugging and testing.~%# float64[3] c3~%# float64[3] t0~%# float64[3] t1~%# float64[3] t2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Tags>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     8
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'tags) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Tags>))
  "Converts a ROS message object to a list"
  (cl:list 'Tags
    (cl:cons ':header (header msg))
    (cl:cons ':image_width (image_width msg))
    (cl:cons ':image_height (image_height msg))
    (cl:cons ':angle_of_view (angle_of_view msg))
    (cl:cons ':tag_count (tag_count msg))
    (cl:cons ':tags (tags msg))
))
