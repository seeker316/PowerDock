// Auto-generated. Do not edit!

// (in-package plutodrone.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Drone_stats {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.accX = null;
      this.accY = null;
      this.accZ = null;
      this.gyroX = null;
      this.gyroY = null;
      this.gyroZ = null;
      this.magX = null;
      this.magY = null;
      this.magZ = null;
      this.roll = null;
      this.pitch = null;
      this.yaw = null;
      this.alt = null;
      this.battery = null;
      this.rssi = null;
      this.a1 = null;
      this.a2 = null;
      this.a3 = null;
    }
    else {
      if (initObj.hasOwnProperty('accX')) {
        this.accX = initObj.accX
      }
      else {
        this.accX = 0.0;
      }
      if (initObj.hasOwnProperty('accY')) {
        this.accY = initObj.accY
      }
      else {
        this.accY = 0.0;
      }
      if (initObj.hasOwnProperty('accZ')) {
        this.accZ = initObj.accZ
      }
      else {
        this.accZ = 0.0;
      }
      if (initObj.hasOwnProperty('gyroX')) {
        this.gyroX = initObj.gyroX
      }
      else {
        this.gyroX = 0.0;
      }
      if (initObj.hasOwnProperty('gyroY')) {
        this.gyroY = initObj.gyroY
      }
      else {
        this.gyroY = 0.0;
      }
      if (initObj.hasOwnProperty('gyroZ')) {
        this.gyroZ = initObj.gyroZ
      }
      else {
        this.gyroZ = 0.0;
      }
      if (initObj.hasOwnProperty('magX')) {
        this.magX = initObj.magX
      }
      else {
        this.magX = 0.0;
      }
      if (initObj.hasOwnProperty('magY')) {
        this.magY = initObj.magY
      }
      else {
        this.magY = 0.0;
      }
      if (initObj.hasOwnProperty('magZ')) {
        this.magZ = initObj.magZ
      }
      else {
        this.magZ = 0.0;
      }
      if (initObj.hasOwnProperty('roll')) {
        this.roll = initObj.roll
      }
      else {
        this.roll = 0.0;
      }
      if (initObj.hasOwnProperty('pitch')) {
        this.pitch = initObj.pitch
      }
      else {
        this.pitch = 0.0;
      }
      if (initObj.hasOwnProperty('yaw')) {
        this.yaw = initObj.yaw
      }
      else {
        this.yaw = 0.0;
      }
      if (initObj.hasOwnProperty('alt')) {
        this.alt = initObj.alt
      }
      else {
        this.alt = 0.0;
      }
      if (initObj.hasOwnProperty('battery')) {
        this.battery = initObj.battery
      }
      else {
        this.battery = 0.0;
      }
      if (initObj.hasOwnProperty('rssi')) {
        this.rssi = initObj.rssi
      }
      else {
        this.rssi = 0.0;
      }
      if (initObj.hasOwnProperty('a1')) {
        this.a1 = initObj.a1
      }
      else {
        this.a1 = 0.0;
      }
      if (initObj.hasOwnProperty('a2')) {
        this.a2 = initObj.a2
      }
      else {
        this.a2 = 0.0;
      }
      if (initObj.hasOwnProperty('a3')) {
        this.a3 = initObj.a3
      }
      else {
        this.a3 = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Drone_stats
    // Serialize message field [accX]
    bufferOffset = _serializer.float32(obj.accX, buffer, bufferOffset);
    // Serialize message field [accY]
    bufferOffset = _serializer.float32(obj.accY, buffer, bufferOffset);
    // Serialize message field [accZ]
    bufferOffset = _serializer.float32(obj.accZ, buffer, bufferOffset);
    // Serialize message field [gyroX]
    bufferOffset = _serializer.float32(obj.gyroX, buffer, bufferOffset);
    // Serialize message field [gyroY]
    bufferOffset = _serializer.float32(obj.gyroY, buffer, bufferOffset);
    // Serialize message field [gyroZ]
    bufferOffset = _serializer.float32(obj.gyroZ, buffer, bufferOffset);
    // Serialize message field [magX]
    bufferOffset = _serializer.float32(obj.magX, buffer, bufferOffset);
    // Serialize message field [magY]
    bufferOffset = _serializer.float32(obj.magY, buffer, bufferOffset);
    // Serialize message field [magZ]
    bufferOffset = _serializer.float32(obj.magZ, buffer, bufferOffset);
    // Serialize message field [roll]
    bufferOffset = _serializer.float32(obj.roll, buffer, bufferOffset);
    // Serialize message field [pitch]
    bufferOffset = _serializer.float32(obj.pitch, buffer, bufferOffset);
    // Serialize message field [yaw]
    bufferOffset = _serializer.float32(obj.yaw, buffer, bufferOffset);
    // Serialize message field [alt]
    bufferOffset = _serializer.float32(obj.alt, buffer, bufferOffset);
    // Serialize message field [battery]
    bufferOffset = _serializer.float32(obj.battery, buffer, bufferOffset);
    // Serialize message field [rssi]
    bufferOffset = _serializer.float32(obj.rssi, buffer, bufferOffset);
    // Serialize message field [a1]
    bufferOffset = _serializer.float32(obj.a1, buffer, bufferOffset);
    // Serialize message field [a2]
    bufferOffset = _serializer.float32(obj.a2, buffer, bufferOffset);
    // Serialize message field [a3]
    bufferOffset = _serializer.float32(obj.a3, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Drone_stats
    let len;
    let data = new Drone_stats(null);
    // Deserialize message field [accX]
    data.accX = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [accY]
    data.accY = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [accZ]
    data.accZ = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [gyroX]
    data.gyroX = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [gyroY]
    data.gyroY = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [gyroZ]
    data.gyroZ = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [magX]
    data.magX = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [magY]
    data.magY = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [magZ]
    data.magZ = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [roll]
    data.roll = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pitch]
    data.pitch = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [yaw]
    data.yaw = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [alt]
    data.alt = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [battery]
    data.battery = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [rssi]
    data.rssi = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [a1]
    data.a1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [a2]
    data.a2 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [a3]
    data.a3 = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 72;
  }

  static datatype() {
    // Returns string type for a message object
    return 'plutodrone/Drone_stats';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fe9c2a462dfb9d7ddfc039b6e8648a47';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Drone_stats.msg
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Drone_stats(null);
    if (msg.accX !== undefined) {
      resolved.accX = msg.accX;
    }
    else {
      resolved.accX = 0.0
    }

    if (msg.accY !== undefined) {
      resolved.accY = msg.accY;
    }
    else {
      resolved.accY = 0.0
    }

    if (msg.accZ !== undefined) {
      resolved.accZ = msg.accZ;
    }
    else {
      resolved.accZ = 0.0
    }

    if (msg.gyroX !== undefined) {
      resolved.gyroX = msg.gyroX;
    }
    else {
      resolved.gyroX = 0.0
    }

    if (msg.gyroY !== undefined) {
      resolved.gyroY = msg.gyroY;
    }
    else {
      resolved.gyroY = 0.0
    }

    if (msg.gyroZ !== undefined) {
      resolved.gyroZ = msg.gyroZ;
    }
    else {
      resolved.gyroZ = 0.0
    }

    if (msg.magX !== undefined) {
      resolved.magX = msg.magX;
    }
    else {
      resolved.magX = 0.0
    }

    if (msg.magY !== undefined) {
      resolved.magY = msg.magY;
    }
    else {
      resolved.magY = 0.0
    }

    if (msg.magZ !== undefined) {
      resolved.magZ = msg.magZ;
    }
    else {
      resolved.magZ = 0.0
    }

    if (msg.roll !== undefined) {
      resolved.roll = msg.roll;
    }
    else {
      resolved.roll = 0.0
    }

    if (msg.pitch !== undefined) {
      resolved.pitch = msg.pitch;
    }
    else {
      resolved.pitch = 0.0
    }

    if (msg.yaw !== undefined) {
      resolved.yaw = msg.yaw;
    }
    else {
      resolved.yaw = 0.0
    }

    if (msg.alt !== undefined) {
      resolved.alt = msg.alt;
    }
    else {
      resolved.alt = 0.0
    }

    if (msg.battery !== undefined) {
      resolved.battery = msg.battery;
    }
    else {
      resolved.battery = 0.0
    }

    if (msg.rssi !== undefined) {
      resolved.rssi = msg.rssi;
    }
    else {
      resolved.rssi = 0.0
    }

    if (msg.a1 !== undefined) {
      resolved.a1 = msg.a1;
    }
    else {
      resolved.a1 = 0.0
    }

    if (msg.a2 !== undefined) {
      resolved.a2 = msg.a2;
    }
    else {
      resolved.a2 = 0.0
    }

    if (msg.a3 !== undefined) {
      resolved.a3 = msg.a3;
    }
    else {
      resolved.a3 = 0.0
    }

    return resolved;
    }
};

module.exports = Drone_stats;
