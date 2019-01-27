;; Auto-generated. Do not edit!


(when (boundp 'custom_msgs::coord)
  (if (not (find-package "CUSTOM_MSGS"))
    (make-package "CUSTOM_MSGS"))
  (shadow 'coord (find-package "CUSTOM_MSGS")))
(unless (find-package "CUSTOM_MSGS::COORD")
  (make-package "CUSTOM_MSGS::COORD"))

(in-package "ROS")
;;//! \htmlinclude coord.msg.html


(defclass custom_msgs::coord
  :super ros::object
  :slots (_x _y _z ))

(defmethod custom_msgs::coord
  (:init
   (&key
    ((:x __x) 0)
    ((:y __y) 0)
    ((:z __z) 0)
    )
   (send-super :init)
   (setq _x (round __x))
   (setq _y (round __y))
   (setq _z (round __z))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:serialization-length
   ()
   (+
    ;; int32 _x
    4
    ;; int32 _y
    4
    ;; int32 _z
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _x
       (write-long _x s)
     ;; int32 _y
       (write-long _y s)
     ;; int32 _z
       (write-long _z s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _x
     (setq _x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _y
     (setq _y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _z
     (setq _z (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get custom_msgs::coord :md5sum-) "3cb41a2c4416de195dbb95b7777a06fb")
(setf (get custom_msgs::coord :datatype-) "custom_msgs/coord")
(setf (get custom_msgs::coord :definition-)
      "int32 x
int32 y
int32 z

")



(provide :custom_msgs/coord "3cb41a2c4416de195dbb95b7777a06fb")


