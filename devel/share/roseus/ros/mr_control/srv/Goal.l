;; Auto-generated. Do not edit!


(when (boundp 'mr_control::Goal)
  (if (not (find-package "MR_CONTROL"))
    (make-package "MR_CONTROL"))
  (shadow 'Goal (find-package "MR_CONTROL")))
(unless (find-package "MR_CONTROL::GOAL")
  (make-package "MR_CONTROL::GOAL"))
(unless (find-package "MR_CONTROL::GOALREQUEST")
  (make-package "MR_CONTROL::GOALREQUEST"))
(unless (find-package "MR_CONTROL::GOALRESPONSE")
  (make-package "MR_CONTROL::GOALRESPONSE"))

(in-package "ROS")





(defclass mr_control::GoalRequest
  :super ros::object
  :slots ())

(defmethod mr_control::GoalRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass mr_control::GoalResponse
  :super ros::object
  :slots (_success _message ))

(defmethod mr_control::GoalResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass mr_control::Goal
  :super ros::object
  :slots ())

(setf (get mr_control::Goal :md5sum-) "937c9679a518e3a18d831e57125ea522")
(setf (get mr_control::Goal :datatype-) "mr_control/Goal")
(setf (get mr_control::Goal :request) mr_control::GoalRequest)
(setf (get mr_control::Goal :response) mr_control::GoalResponse)

(defmethod mr_control::GoalRequest
  (:response () (instance mr_control::GoalResponse :init)))

(setf (get mr_control::GoalRequest :md5sum-) "937c9679a518e3a18d831e57125ea522")
(setf (get mr_control::GoalRequest :datatype-) "mr_control/GoalRequest")
(setf (get mr_control::GoalRequest :definition-)
      "---
bool success
string message

")

(setf (get mr_control::GoalResponse :md5sum-) "937c9679a518e3a18d831e57125ea522")
(setf (get mr_control::GoalResponse :datatype-) "mr_control/GoalResponse")
(setf (get mr_control::GoalResponse :definition-)
      "---
bool success
string message

")



(provide :mr_control/Goal "937c9679a518e3a18d831e57125ea522")


