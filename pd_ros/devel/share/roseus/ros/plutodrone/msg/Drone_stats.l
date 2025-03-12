;; Auto-generated. Do not edit!


(when (boundp 'plutodrone::Drone_stats)
  (if (not (find-package "PLUTODRONE"))
    (make-package "PLUTODRONE"))
  (shadow 'Drone_stats (find-package "PLUTODRONE")))
(unless (find-package "PLUTODRONE::DRONE_STATS")
  (make-package "PLUTODRONE::DRONE_STATS"))

(in-package "ROS")
;;//! \htmlinclude Drone_stats.msg.html


(defclass plutodrone::Drone_stats
  :super ros::object
  :slots (_accX _accY _accZ _gyroX _gyroY _gyroZ _magX _magY _magZ _roll _pitch _yaw _alt _battery _rssi _a1 _a2 _a3 ))

(defmethod plutodrone::Drone_stats
  (:init
   (&key
    ((:accX __accX) 0.0)
    ((:accY __accY) 0.0)
    ((:accZ __accZ) 0.0)
    ((:gyroX __gyroX) 0.0)
    ((:gyroY __gyroY) 0.0)
    ((:gyroZ __gyroZ) 0.0)
    ((:magX __magX) 0.0)
    ((:magY __magY) 0.0)
    ((:magZ __magZ) 0.0)
    ((:roll __roll) 0.0)
    ((:pitch __pitch) 0.0)
    ((:yaw __yaw) 0.0)
    ((:alt __alt) 0.0)
    ((:battery __battery) 0.0)
    ((:rssi __rssi) 0.0)
    ((:a1 __a1) 0.0)
    ((:a2 __a2) 0.0)
    ((:a3 __a3) 0.0)
    )
   (send-super :init)
   (setq _accX (float __accX))
   (setq _accY (float __accY))
   (setq _accZ (float __accZ))
   (setq _gyroX (float __gyroX))
   (setq _gyroY (float __gyroY))
   (setq _gyroZ (float __gyroZ))
   (setq _magX (float __magX))
   (setq _magY (float __magY))
   (setq _magZ (float __magZ))
   (setq _roll (float __roll))
   (setq _pitch (float __pitch))
   (setq _yaw (float __yaw))
   (setq _alt (float __alt))
   (setq _battery (float __battery))
   (setq _rssi (float __rssi))
   (setq _a1 (float __a1))
   (setq _a2 (float __a2))
   (setq _a3 (float __a3))
   self)
  (:accX
   (&optional __accX)
   (if __accX (setq _accX __accX)) _accX)
  (:accY
   (&optional __accY)
   (if __accY (setq _accY __accY)) _accY)
  (:accZ
   (&optional __accZ)
   (if __accZ (setq _accZ __accZ)) _accZ)
  (:gyroX
   (&optional __gyroX)
   (if __gyroX (setq _gyroX __gyroX)) _gyroX)
  (:gyroY
   (&optional __gyroY)
   (if __gyroY (setq _gyroY __gyroY)) _gyroY)
  (:gyroZ
   (&optional __gyroZ)
   (if __gyroZ (setq _gyroZ __gyroZ)) _gyroZ)
  (:magX
   (&optional __magX)
   (if __magX (setq _magX __magX)) _magX)
  (:magY
   (&optional __magY)
   (if __magY (setq _magY __magY)) _magY)
  (:magZ
   (&optional __magZ)
   (if __magZ (setq _magZ __magZ)) _magZ)
  (:roll
   (&optional __roll)
   (if __roll (setq _roll __roll)) _roll)
  (:pitch
   (&optional __pitch)
   (if __pitch (setq _pitch __pitch)) _pitch)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:alt
   (&optional __alt)
   (if __alt (setq _alt __alt)) _alt)
  (:battery
   (&optional __battery)
   (if __battery (setq _battery __battery)) _battery)
  (:rssi
   (&optional __rssi)
   (if __rssi (setq _rssi __rssi)) _rssi)
  (:a1
   (&optional __a1)
   (if __a1 (setq _a1 __a1)) _a1)
  (:a2
   (&optional __a2)
   (if __a2 (setq _a2 __a2)) _a2)
  (:a3
   (&optional __a3)
   (if __a3 (setq _a3 __a3)) _a3)
  (:serialization-length
   ()
   (+
    ;; float32 _accX
    4
    ;; float32 _accY
    4
    ;; float32 _accZ
    4
    ;; float32 _gyroX
    4
    ;; float32 _gyroY
    4
    ;; float32 _gyroZ
    4
    ;; float32 _magX
    4
    ;; float32 _magY
    4
    ;; float32 _magZ
    4
    ;; float32 _roll
    4
    ;; float32 _pitch
    4
    ;; float32 _yaw
    4
    ;; float32 _alt
    4
    ;; float32 _battery
    4
    ;; float32 _rssi
    4
    ;; float32 _a1
    4
    ;; float32 _a2
    4
    ;; float32 _a3
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _accX
       (sys::poke _accX (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _accY
       (sys::poke _accY (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _accZ
       (sys::poke _accZ (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _gyroX
       (sys::poke _gyroX (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _gyroY
       (sys::poke _gyroY (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _gyroZ
       (sys::poke _gyroZ (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _magX
       (sys::poke _magX (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _magY
       (sys::poke _magY (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _magZ
       (sys::poke _magZ (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _roll
       (sys::poke _roll (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pitch
       (sys::poke _pitch (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _alt
       (sys::poke _alt (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _battery
       (sys::poke _battery (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rssi
       (sys::poke _rssi (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _a1
       (sys::poke _a1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _a2
       (sys::poke _a2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _a3
       (sys::poke _a3 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _accX
     (setq _accX (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _accY
     (setq _accY (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _accZ
     (setq _accZ (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _gyroX
     (setq _gyroX (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _gyroY
     (setq _gyroY (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _gyroZ
     (setq _gyroZ (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _magX
     (setq _magX (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _magY
     (setq _magY (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _magZ
     (setq _magZ (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _roll
     (setq _roll (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pitch
     (setq _pitch (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yaw
     (setq _yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _alt
     (setq _alt (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _battery
     (setq _battery (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rssi
     (setq _rssi (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _a1
     (setq _a1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _a2
     (setq _a2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _a3
     (setq _a3 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get plutodrone::Drone_stats :md5sum-) "fe9c2a462dfb9d7ddfc039b6e8648a47")
(setf (get plutodrone::Drone_stats :datatype-) "plutodrone/Drone_stats")
(setf (get plutodrone::Drone_stats :definition-)
      "# Drone_stats.msg
# Accelerometer data
float32 accX
float32 accY
float32 accZ

# Gyroscope data
float32 gyroX
float32 gyroY
float32 gyroZ

# Magnetometer data
float32 magX
float32 magY
float32 magZ

# Orientation
float32 roll
float32 pitch
float32 yaw

# Altitude and battery data
float32 alt
float32 battery
float32 rssi

# Anchor data
float32 a1
float32 a2
float32 a3

")



(provide :plutodrone/Drone_stats "fe9c2a462dfb9d7ddfc039b6e8648a47")


