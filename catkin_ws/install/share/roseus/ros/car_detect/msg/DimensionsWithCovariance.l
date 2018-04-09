;; Auto-generated. Do not edit!


(when (boundp 'car_detect::DimensionsWithCovariance)
  (if (not (find-package "CAR_DETECT"))
    (make-package "CAR_DETECT"))
  (shadow 'DimensionsWithCovariance (find-package "CAR_DETECT")))
(unless (find-package "CAR_DETECT::DIMENSIONSWITHCOVARIANCE")
  (make-package "CAR_DETECT::DIMENSIONSWITHCOVARIANCE"))

(in-package "ROS")
;;//! \htmlinclude DimensionsWithCovariance.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass car_detect::DimensionsWithCovariance
  :super ros::object
  :slots (_dimensions _covariance ))

(defmethod car_detect::DimensionsWithCovariance
  (:init
   (&key
    ((:dimensions __dimensions) (instance geometry_msgs::Vector3 :init))
    ((:covariance __covariance) (make-array 9 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _dimensions __dimensions)
   (setq _covariance __covariance)
   self)
  (:dimensions
   (&rest __dimensions)
   (if (keywordp (car __dimensions))
       (send* _dimensions __dimensions)
     (progn
       (if __dimensions (setq _dimensions (car __dimensions)))
       _dimensions)))
  (:covariance
   (&optional __covariance)
   (if __covariance (setq _covariance __covariance)) _covariance)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Vector3 _dimensions
    (send _dimensions :serialization-length)
    ;; float64[9] _covariance
    (* 8    9)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Vector3 _dimensions
       (send _dimensions :serialize s)
     ;; float64[9] _covariance
     (dotimes (i 9)
       (sys::poke (elt _covariance i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Vector3 _dimensions
     (send _dimensions :deserialize buf ptr-) (incf ptr- (send _dimensions :serialization-length))
   ;; float64[9] _covariance
   (dotimes (i (length _covariance))
     (setf (elt _covariance i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get car_detect::DimensionsWithCovariance :md5sum-) "f47d667774dd07bc75421f9c0a4886e7")
(setf (get car_detect::DimensionsWithCovariance :datatype-) "car_detect/DimensionsWithCovariance")
(setf (get car_detect::DimensionsWithCovariance :definition-)
      "geometry_msgs/Vector3            dimensions     # sizes of the bounding box
float64[9] covariance

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :car_detect/DimensionsWithCovariance "f47d667774dd07bc75421f9c0a4886e7")


