;; Auto-generated. Do not edit!


(when (boundp 'plutodrone::aruco)
  (if (not (find-package "PLUTODRONE"))
    (make-package "PLUTODRONE"))
  (shadow 'aruco (find-package "PLUTODRONE")))
(unless (find-package "PLUTODRONE::ARUCO")
  (make-package "PLUTODRONE::ARUCO"))
(unless (find-package "PLUTODRONE::ARUCOREQUEST")
  (make-package "PLUTODRONE::ARUCOREQUEST"))
(unless (find-package "PLUTODRONE::ARUCORESPONSE")
  (make-package "PLUTODRONE::ARUCORESPONSE"))

(in-package "ROS")





(defclass plutodrone::arucoRequest
  :super ros::object
  :slots (_size ))

(defmethod plutodrone::arucoRequest
  (:init
   (&key
    ((:size __size) 0)
    )
   (send-super :init)
   (setq _size (round __size))
   self)
  (:size
   (&optional __size)
   (if __size (setq _size __size)) _size)
  (:serialization-length
   ()
   (+
    ;; int32 _size
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _size
       (write-long _size s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _size
     (setq _size (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass plutodrone::arucoResponse
  :super ros::object
  :slots (_data ))

(defmethod plutodrone::arucoResponse
  (:init
   (&key
    ((:data __data) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _data __data)
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; int16[] _data
    (* 2    (length _data)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16[] _data
     (write-long (length _data) s)
     (dotimes (i (length _data))
       (write-word (elt _data i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _data i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;;
   self)
  )

(defclass plutodrone::aruco
  :super ros::object
  :slots ())

(setf (get plutodrone::aruco :md5sum-) "c31722818caef8e97292f53a960b806f")
(setf (get plutodrone::aruco :datatype-) "plutodrone/aruco")
(setf (get plutodrone::aruco :request) plutodrone::arucoRequest)
(setf (get plutodrone::aruco :response) plutodrone::arucoResponse)

(defmethod plutodrone::arucoRequest
  (:response () (instance plutodrone::arucoResponse :init)))

(setf (get plutodrone::arucoRequest :md5sum-) "c31722818caef8e97292f53a960b806f")
(setf (get plutodrone::arucoRequest :datatype-) "plutodrone/arucoRequest")
(setf (get plutodrone::arucoRequest :definition-)
      "int32 size  # Request: The number of elements in the array
---
int16[] data  # Response: The array of int16 values

")

(setf (get plutodrone::arucoResponse :md5sum-) "c31722818caef8e97292f53a960b806f")
(setf (get plutodrone::arucoResponse :datatype-) "plutodrone/arucoResponse")
(setf (get plutodrone::arucoResponse :definition-)
      "int32 size  # Request: The number of elements in the array
---
int16[] data  # Response: The array of int16 values

")



(provide :plutodrone/aruco "c31722818caef8e97292f53a960b806f")


