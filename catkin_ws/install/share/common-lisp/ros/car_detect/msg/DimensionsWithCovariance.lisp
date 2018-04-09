; Auto-generated. Do not edit!


(cl:in-package car_detect-msg)


;//! \htmlinclude DimensionsWithCovariance.msg.html

(cl:defclass <DimensionsWithCovariance> (roslisp-msg-protocol:ros-message)
  ((dimensions
    :reader dimensions
    :initarg :dimensions
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (covariance
    :reader covariance
    :initarg :covariance
    :type (cl:vector cl:float)
   :initform (cl:make-array 9 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass DimensionsWithCovariance (<DimensionsWithCovariance>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DimensionsWithCovariance>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DimensionsWithCovariance)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name car_detect-msg:<DimensionsWithCovariance> is deprecated: use car_detect-msg:DimensionsWithCovariance instead.")))

(cl:ensure-generic-function 'dimensions-val :lambda-list '(m))
(cl:defmethod dimensions-val ((m <DimensionsWithCovariance>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader car_detect-msg:dimensions-val is deprecated.  Use car_detect-msg:dimensions instead.")
  (dimensions m))

(cl:ensure-generic-function 'covariance-val :lambda-list '(m))
(cl:defmethod covariance-val ((m <DimensionsWithCovariance>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader car_detect-msg:covariance-val is deprecated.  Use car_detect-msg:covariance instead.")
  (covariance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DimensionsWithCovariance>) ostream)
  "Serializes a message object of type '<DimensionsWithCovariance>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'dimensions) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'covariance))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DimensionsWithCovariance>) istream)
  "Deserializes a message object of type '<DimensionsWithCovariance>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'dimensions) istream)
  (cl:setf (cl:slot-value msg 'covariance) (cl:make-array 9))
  (cl:let ((vals (cl:slot-value msg 'covariance)))
    (cl:dotimes (i 9)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DimensionsWithCovariance>)))
  "Returns string type for a message object of type '<DimensionsWithCovariance>"
  "car_detect/DimensionsWithCovariance")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DimensionsWithCovariance)))
  "Returns string type for a message object of type 'DimensionsWithCovariance"
  "car_detect/DimensionsWithCovariance")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DimensionsWithCovariance>)))
  "Returns md5sum for a message object of type '<DimensionsWithCovariance>"
  "f47d667774dd07bc75421f9c0a4886e7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DimensionsWithCovariance)))
  "Returns md5sum for a message object of type 'DimensionsWithCovariance"
  "f47d667774dd07bc75421f9c0a4886e7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DimensionsWithCovariance>)))
  "Returns full string definition for message of type '<DimensionsWithCovariance>"
  (cl:format cl:nil "geometry_msgs/Vector3            dimensions     # sizes of the bounding box~%float64[9] covariance~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DimensionsWithCovariance)))
  "Returns full string definition for message of type 'DimensionsWithCovariance"
  (cl:format cl:nil "geometry_msgs/Vector3            dimensions     # sizes of the bounding box~%float64[9] covariance~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DimensionsWithCovariance>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'dimensions))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'covariance) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DimensionsWithCovariance>))
  "Converts a ROS message object to a list"
  (cl:list 'DimensionsWithCovariance
    (cl:cons ':dimensions (dimensions msg))
    (cl:cons ':covariance (covariance msg))
))
