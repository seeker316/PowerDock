;; Auto-generated. Do not edit!


(when (boundp 'plutodrone::SetPos)
  (if (not (find-package "PLUTODRONE"))
    (make-package "PLUTODRONE"))
  (shadow 'SetPos (find-package "PLUTODRONE")))
(unless (find-package "PLUTODRONE::SETPOS")
  (make-package "PLUTODRONE::SETPOS"))
(unless (find-package "PLUTODRONE::SETPOSREQUEST")
  (make-package "PLUTODRONE::SETPOSREQUEST"))
(unless (find-package "PLUTODRONE::SETPOSRESPONSE")
  (make-package "PLUTODRONE::SETPOSRESPONSE"))

(in-package "ROS")





(defclass plutodrone::SetPosRequest
  :super ros::object
  :slots (_pos_x _pos_y _pos_z ))

(defmethod plutodrone::SetPosRequest
  (:init
   (&key
    ((:pos_x __pos_x) 0.0)
    ((:pos_y __pos_y) 0.0)
    ((:pos_z __pos_z) 0.0)
    )
   (send-super :init)
   (setq _pos_x (float __pos_x))
   (setq _pos_y (float __pos_y))
   (setq _pos_z (float __pos_z))
   self)
  (:pos_x
   (&optional __pos_x)
   (if __pos_x (setq _pos_x __pos_x)) _pos_x)
  (:pos_y
   (&optional __pos_y)
   (if __pos_y (setq _pos_y __pos_y)) _pos_y)
  (:pos_z
   (&optional __pos_z)
   (if __pos_z (setq _pos_z __pos_z)) _pos_z)
  (:serialization-length
   ()
   (+
    ;; float64 _pos_x
    8
    ;; float64 _pos_y
    8
    ;; float64 _pos_z
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _pos_x
       (sys::poke _pos_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _pos_y
       (sys::poke _pos_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _pos_z
       (sys::poke _pos_z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _pos_x
     (setq _pos_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _pos_y
     (setq _pos_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _pos_z
     (setq _pos_z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass plutodrone::SetPosResponse
  :super ros::object
  :slots (_set_x _set_y _set_z ))

(defmethod plutodrone::SetPosResponse
  (:init
   (&key
    ((:set_x __set_x) 0.0)
    ((:set_y __set_y) 0.0)
    ((:set_z __set_z) 0.0)
    )
   (send-super :init)
   (setq _set_x (float __set_x))
   (setq _set_y (float __set_y))
   (setq _set_z (float __set_z))
   self)
  (:set_x
   (&optional __set_x)
   (if __set_x (setq _set_x __set_x)) _set_x)
  (:set_y
   (&optional __set_y)
   (if __set_y (setq _set_y __set_y)) _set_y)
  (:set_z
   (&optional __set_z)
   (if __set_z (setq _set_z __set_z)) _set_z)
  (:serialization-length
   ()
   (+
    ;; float64 _set_x
    8
    ;; float64 _set_y
    8
    ;; float64 _set_z
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _set_x
       (sys::poke _set_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _set_y
       (sys::poke _set_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _set_z
       (sys::poke _set_z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _set_x
     (setq _set_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _set_y
     (setq _set_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _set_z
     (setq _set_z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass plutodrone::SetPos
  :super ros::object
  :slots ())

(setf (get plutodrone::SetPos :md5sum-) "0887ae55fe5cc3ab69632861e1865920")
(setf (get plutodrone::SetPos :datatype-) "plutodrone/SetPos")
(setf (get plutodrone::SetPos :request) plutodrone::SetPosRequest)
(setf (get plutodrone::SetPos :response) plutodrone::SetPosResponse)

(defmethod plutodrone::SetPosRequest
  (:response () (instance plutodrone::SetPosResponse :init)))

(setf (get plutodrone::SetPosRequest :md5sum-) "0887ae55fe5cc3ab69632861e1865920")
(setf (get plutodrone::SetPosRequest :datatype-) "plutodrone/SetPosRequest")
(setf (get plutodrone::SetPosRequest :definition-)
      "# SetPos.srv

float64 pos_x
float64 pos_y
float64 pos_z
---
float64 set_x
float64 set_y
float64 set_z

")

(setf (get plutodrone::SetPosResponse :md5sum-) "0887ae55fe5cc3ab69632861e1865920")
(setf (get plutodrone::SetPosResponse :datatype-) "plutodrone/SetPosResponse")
(setf (get plutodrone::SetPosResponse :definition-)
      "# SetPos.srv

float64 pos_x
float64 pos_y
float64 pos_z
---
float64 set_x
float64 set_y
float64 set_z

")



(provide :plutodrone/SetPos "0887ae55fe5cc3ab69632861e1865920")


