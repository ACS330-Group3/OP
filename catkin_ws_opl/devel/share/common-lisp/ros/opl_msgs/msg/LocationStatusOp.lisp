; Auto-generated. Do not edit!


(cl:in-package opl_msgs-msg)


;//! \htmlinclude LocationStatusOp.msg.html

(cl:defclass <LocationStatusOp> (roslisp-msg-protocol:ros-message)
  ((location
    :reader location
    :initarg :location
    :type cl:string
    :initform "")
   (cube_at_ds
    :reader cube_at_ds
    :initarg :cube_at_ds
    :type cl:boolean
    :initform cl:nil)
   (op_cube_id
    :reader op_cube_id
    :initarg :op_cube_id
    :type cl:string
    :initform ""))
)

(cl:defclass LocationStatusOp (<LocationStatusOp>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LocationStatusOp>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LocationStatusOp)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name opl_msgs-msg:<LocationStatusOp> is deprecated: use opl_msgs-msg:LocationStatusOp instead.")))

(cl:ensure-generic-function 'location-val :lambda-list '(m))
(cl:defmethod location-val ((m <LocationStatusOp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opl_msgs-msg:location-val is deprecated.  Use opl_msgs-msg:location instead.")
  (location m))

(cl:ensure-generic-function 'cube_at_ds-val :lambda-list '(m))
(cl:defmethod cube_at_ds-val ((m <LocationStatusOp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opl_msgs-msg:cube_at_ds-val is deprecated.  Use opl_msgs-msg:cube_at_ds instead.")
  (cube_at_ds m))

(cl:ensure-generic-function 'op_cube_id-val :lambda-list '(m))
(cl:defmethod op_cube_id-val ((m <LocationStatusOp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opl_msgs-msg:op_cube_id-val is deprecated.  Use opl_msgs-msg:op_cube_id instead.")
  (op_cube_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LocationStatusOp>) ostream)
  "Serializes a message object of type '<LocationStatusOp>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'location))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'location))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'cube_at_ds) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'op_cube_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'op_cube_id))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LocationStatusOp>) istream)
  "Deserializes a message object of type '<LocationStatusOp>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'location) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'location) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'cube_at_ds) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'op_cube_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'op_cube_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LocationStatusOp>)))
  "Returns string type for a message object of type '<LocationStatusOp>"
  "opl_msgs/LocationStatusOp")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LocationStatusOp)))
  "Returns string type for a message object of type 'LocationStatusOp"
  "opl_msgs/LocationStatusOp")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LocationStatusOp>)))
  "Returns md5sum for a message object of type '<LocationStatusOp>"
  "d7e59fda5792b384bfe9ef98e01f915a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LocationStatusOp)))
  "Returns md5sum for a message object of type 'LocationStatusOp"
  "d7e59fda5792b384bfe9ef98e01f915a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LocationStatusOp>)))
  "Returns full string definition for message of type '<LocationStatusOp>"
  (cl:format cl:nil "string location~%bool cube_at_ds~%string op_cube_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LocationStatusOp)))
  "Returns full string definition for message of type 'LocationStatusOp"
  (cl:format cl:nil "string location~%bool cube_at_ds~%string op_cube_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LocationStatusOp>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'location))
     1
     4 (cl:length (cl:slot-value msg 'op_cube_id))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LocationStatusOp>))
  "Converts a ROS message object to a list"
  (cl:list 'LocationStatusOp
    (cl:cons ':location (location msg))
    (cl:cons ':cube_at_ds (cube_at_ds msg))
    (cl:cons ':op_cube_id (op_cube_id msg))
))
