; Auto-generated. Do not edit!


(cl:in-package plutodrone-srv)


;//! \htmlinclude aruco-request.msg.html

(cl:defclass <aruco-request> (roslisp-msg-protocol:ros-message)
  ((size
    :reader size
    :initarg :size
    :type cl:integer
    :initform 0))
)

(cl:defclass aruco-request (<aruco-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <aruco-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'aruco-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name plutodrone-srv:<aruco-request> is deprecated: use plutodrone-srv:aruco-request instead.")))

(cl:ensure-generic-function 'size-val :lambda-list '(m))
(cl:defmethod size-val ((m <aruco-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plutodrone-srv:size-val is deprecated.  Use plutodrone-srv:size instead.")
  (size m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <aruco-request>) ostream)
  "Serializes a message object of type '<aruco-request>"
  (cl:let* ((signed (cl:slot-value msg 'size)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <aruco-request>) istream)
  "Deserializes a message object of type '<aruco-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'size) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<aruco-request>)))
  "Returns string type for a service object of type '<aruco-request>"
  "plutodrone/arucoRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'aruco-request)))
  "Returns string type for a service object of type 'aruco-request"
  "plutodrone/arucoRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<aruco-request>)))
  "Returns md5sum for a message object of type '<aruco-request>"
  "c31722818caef8e97292f53a960b806f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'aruco-request)))
  "Returns md5sum for a message object of type 'aruco-request"
  "c31722818caef8e97292f53a960b806f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<aruco-request>)))
  "Returns full string definition for message of type '<aruco-request>"
  (cl:format cl:nil "int32 size  # Request: The number of elements in the array~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'aruco-request)))
  "Returns full string definition for message of type 'aruco-request"
  (cl:format cl:nil "int32 size  # Request: The number of elements in the array~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <aruco-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <aruco-request>))
  "Converts a ROS message object to a list"
  (cl:list 'aruco-request
    (cl:cons ':size (size msg))
))
;//! \htmlinclude aruco-response.msg.html

(cl:defclass <aruco-response> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass aruco-response (<aruco-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <aruco-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'aruco-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name plutodrone-srv:<aruco-response> is deprecated: use plutodrone-srv:aruco-response instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <aruco-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plutodrone-srv:data-val is deprecated.  Use plutodrone-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <aruco-response>) ostream)
  "Serializes a message object of type '<aruco-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <aruco-response>) istream)
  "Deserializes a message object of type '<aruco-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<aruco-response>)))
  "Returns string type for a service object of type '<aruco-response>"
  "plutodrone/arucoResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'aruco-response)))
  "Returns string type for a service object of type 'aruco-response"
  "plutodrone/arucoResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<aruco-response>)))
  "Returns md5sum for a message object of type '<aruco-response>"
  "c31722818caef8e97292f53a960b806f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'aruco-response)))
  "Returns md5sum for a message object of type 'aruco-response"
  "c31722818caef8e97292f53a960b806f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<aruco-response>)))
  "Returns full string definition for message of type '<aruco-response>"
  (cl:format cl:nil "int16[] data  # Response: The array of int16 values~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'aruco-response)))
  "Returns full string definition for message of type 'aruco-response"
  (cl:format cl:nil "int16[] data  # Response: The array of int16 values~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <aruco-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <aruco-response>))
  "Converts a ROS message object to a list"
  (cl:list 'aruco-response
    (cl:cons ':data (data msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'aruco)))
  'aruco-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'aruco)))
  'aruco-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'aruco)))
  "Returns string type for a service object of type '<aruco>"
  "plutodrone/aruco")