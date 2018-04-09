
(cl:in-package :asdf)

(defsystem "car_detect-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "DimensionsWithCovariance" :depends-on ("_package_DimensionsWithCovariance"))
    (:file "_package_DimensionsWithCovariance" :depends-on ("_package"))
    (:file "TrackedObject" :depends-on ("_package_TrackedObject"))
    (:file "_package_TrackedObject" :depends-on ("_package"))
  ))