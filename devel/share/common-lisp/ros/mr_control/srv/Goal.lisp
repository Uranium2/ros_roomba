; Auto-generated. Do not edit!


(cl:in-package mr_control-srv)


;//! \htmlinclude Goal-request.msg.html

(cl:defclass <Goal-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Goal-request (<Goal-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Goal-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Goal-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mr_control-srv:<Goal-request> is deprecated: use mr_control-srv:Goal-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Goal-request>) ostream)
  "Serializes a message object of type '<Goal-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Goal-request>) istream)
  "Deserializes a message object of type '<Goal-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Goal-request>)))
  "Returns string type for a service object of type '<Goal-request>"
  "mr_control/GoalRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Goal-request)))
  "Returns string type for a service object of type 'Goal-request"
  "mr_control/GoalRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Goal-request>)))
  "Returns md5sum for a message object of type '<Goal-request>"
  "937c9679a518e3a18d831e57125ea522")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Goal-request)))
  "Returns md5sum for a message object of type 'Goal-request"
  "937c9679a518e3a18d831e57125ea522")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Goal-request>)))
  "Returns full string definition for message of type '<Goal-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Goal-request)))
  "Returns full string definition for message of type 'Goal-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Goal-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Goal-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Goal-request
))
;//! \htmlinclude Goal-response.msg.html

(cl:defclass <Goal-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass Goal-response (<Goal-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Goal-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Goal-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mr_control-srv:<Goal-response> is deprecated: use mr_control-srv:Goal-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <Goal-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mr_control-srv:success-val is deprecated.  Use mr_control-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <Goal-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mr_control-srv:message-val is deprecated.  Use mr_control-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Goal-response>) ostream)
  "Serializes a message object of type '<Goal-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Goal-response>) istream)
  "Deserializes a message object of type '<Goal-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Goal-response>)))
  "Returns string type for a service object of type '<Goal-response>"
  "mr_control/GoalResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Goal-response)))
  "Returns string type for a service object of type 'Goal-response"
  "mr_control/GoalResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Goal-response>)))
  "Returns md5sum for a message object of type '<Goal-response>"
  "937c9679a518e3a18d831e57125ea522")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Goal-response)))
  "Returns md5sum for a message object of type 'Goal-response"
  "937c9679a518e3a18d831e57125ea522")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Goal-response>)))
  "Returns full string definition for message of type '<Goal-response>"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Goal-response)))
  "Returns full string definition for message of type 'Goal-response"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Goal-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Goal-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Goal-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Goal)))
  'Goal-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Goal)))
  'Goal-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Goal)))
  "Returns string type for a service object of type '<Goal>"
  "mr_control/Goal")