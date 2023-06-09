;; Auto-generated. Do not edit!


(when (boundp 'motion_model::motion_model_msgs)
  (if (not (find-package "MOTION_MODEL"))
    (make-package "MOTION_MODEL"))
  (shadow 'motion_model_msgs (find-package "MOTION_MODEL")))
(unless (find-package "MOTION_MODEL::MOTION_MODEL_MSGS")
  (make-package "MOTION_MODEL::MOTION_MODEL_MSGS"))

(in-package "ROS")
;;//! \htmlinclude motion_model_msgs.msg.html


(defclass motion_model::motion_model_msgs
  :super ros::object
  :slots (_x _y _yaw ))

(defmethod motion_model::motion_model_msgs
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:yaw __yaw) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _yaw (float __yaw))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:serialization-length
   ()
   (+
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _yaw
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _yaw
     (setq _yaw (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get motion_model::motion_model_msgs :md5sum-) "b71344c0ba01936756dc45bcd89ef437")
(setf (get motion_model::motion_model_msgs :datatype-) "motion_model/motion_model_msgs")
(setf (get motion_model::motion_model_msgs :definition-)
      "float64 x
float64 y
float64 yaw
")



(provide :motion_model/motion_model_msgs "b71344c0ba01936756dc45bcd89ef437")


