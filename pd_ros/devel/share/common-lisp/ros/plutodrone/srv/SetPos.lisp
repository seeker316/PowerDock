; Auto-generated. Do not edit!


(cl:in-package plutodrone-srv)


;//! \htmlinclude SetPos-request.msg.html

(cl:defclass <SetPos-request> (roslisp-msg-protocol:ros-message)
  ((pos_x
    :reader pos_x
    :initarg :pos_x
    :type cl:float
    :initform 0.0)
   (pos_y
    :reader pos_y
    :initarg :pos_y
    :type cl:float
    :initform 0.0)
   (pos_z
    :reader pos_z
    :initarg :pos_z
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetPos-request (<SetPos-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetPos-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetPos-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name plutodrone-srv:<SetPos-request> is deprecated: use plutodrone-srv:SetPos-request instead.")))

(cl:ensure-generic-function 'pos_x-val :lambda-list '(m))
(cl:defmethod pos_x-val ((m <SetPos-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plutodrone-srv:pos_x-val is deprecated.  Use plutodrone-srv:pos_x instead.")
  (pos_x m))

(cl:ensure-generic-function 'pos_y-val :lambda-list '(m))
(cl:defmethod pos_y-val ((m <SetPos-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plutodrone-srv:pos_y-val is deprecated.  Use plutodrone-srv:pos_y instead.")
  (pos_y m))

(cl:ensure-generic-function 'pos_z-val :lambda-list '(m))
(cl:defmethod pos_z-val ((m <SetPos-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plutodrone-srv:pos_z-val is deprecated.  Use plutodrone-srv:pos_z instead.")
  (pos_z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetPos-request>) ostream)
  "Serializes a message object of type '<SetPos-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'pos_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'pos_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'pos_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetPos-request>) istream)
  "Deserializes a message object of type '<SetPos-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pos_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pos_y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pos_z) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetPos-request>)))
  "Returns string type for a service object of type '<SetPos-request>"
  "plutodrone/SetPosRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetPos-request)))
  "Returns string type for a service object of type 'SetPos-request"
  "plutodrone/SetPosRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetPos-request>)))
  "Returns md5sum for a message object of type '<SetPos-request>"
  "0887ae55fe5cc3ab69632861e1865920")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetPos-request)))
  "Returns md5sum for a message object of type 'SetPos-request"
  "0887ae55fe5cc3ab69632861e1865920")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetPos-request>)))
  "Returns full string definition for message of type '<SetPos-request>"
  (cl:format cl:nil "# SetPos.srv~%~%float64 pos_x~%float64 pos_y~%float64 pos_z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetPos-request)))
  "Returns full string definition for message of type 'SetPos-request"
  (cl:format cl:nil "# SetPos.srv~%~%float64 pos_x~%float64 pos_y~%float64 pos_z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetPos-request>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetPos-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetPos-request
    (cl:cons ':pos_x (pos_x msg))
    (cl:cons ':pos_y (pos_y msg))
    (cl:cons ':pos_z (pos_z msg))
))
;//! \htmlinclude SetPos-response.msg.html

(cl:defclass <SetPos-response> (roslisp-msg-protocol:ros-message)
  ((set_x
    :reader set_x
    :initarg :set_x
    :type cl:float
    :initform 0.0)
   (set_y
    :reader set_y
    :initarg :set_y
    :type cl:float
    :initform 0.0)
   (set_z
    :reader set_z
    :initarg :set_z
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetPos-response (<SetPos-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetPos-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetPos-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name plutodrone-srv:<SetPos-response> is deprecated: use plutodrone-srv:SetPos-response instead.")))

(cl:ensure-generic-function 'set_x-val :lambda-list '(m))
(cl:defmethod set_x-val ((m <SetPos-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plutodrone-srv:set_x-val is deprecated.  Use plutodrone-srv:set_x instead.")
  (set_x m))

(cl:ensure-generic-function 'set_y-val :lambda-list '(m))
(cl:defmethod set_y-val ((m <SetPos-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plutodrone-srv:set_y-val is deprecated.  Use plutodrone-srv:set_y instead.")
  (set_y m))

(cl:ensure-generic-function 'set_z-val :lambda-list '(m))
(cl:defmethod set_z-val ((m <SetPos-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plutodrone-srv:set_z-val is deprecated.  Use plutodrone-srv:set_z instead.")
  (set_z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetPos-response>) ostream)
  "Serializes a message object of type '<SetPos-response>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'set_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'set_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'set_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetPos-response>) istream)
  "Deserializes a message object of type '<SetPos-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'set_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'set_y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'set_z) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetPos-response>)))
  "Returns string type for a service object of type '<SetPos-response>"
  "plutodrone/SetPosResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetPos-response)))
  "Returns string type for a service object of type 'SetPos-response"
  "plutodrone/SetPosResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetPos-response>)))
  "Returns md5sum for a message object of type '<SetPos-response>"
  "0887ae55fe5cc3ab69632861e1865920")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetPos-response)))
  "Returns md5sum for a message object of type 'SetPos-response"
  "0887ae55fe5cc3ab69632861e1865920")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetPos-response>)))
  "Returns full string definition for message of type '<SetPos-response>"
  (cl:format cl:nil "float64 set_x~%float64 set_y~%float64 set_z~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetPos-response)))
  "Returns full string definition for message of type 'SetPos-response"
  (cl:format cl:nil "float64 set_x~%float64 set_y~%float64 set_z~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetPos-response>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetPos-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetPos-response
    (cl:cons ':set_x (set_x msg))
    (cl:cons ':set_y (set_y msg))
    (cl:cons ':set_z (set_z msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetPos)))
  'SetPos-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetPos)))
  'SetPos-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetPos)))
  "Returns string type for a service object of type '<SetPos>"
  "plutodrone/SetPos")