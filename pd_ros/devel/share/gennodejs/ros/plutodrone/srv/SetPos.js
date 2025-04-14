// Auto-generated. Do not edit!

// (in-package plutodrone.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetPosRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pos_x = null;
      this.pos_y = null;
      this.pos_z = null;
    }
    else {
      if (initObj.hasOwnProperty('pos_x')) {
        this.pos_x = initObj.pos_x
      }
      else {
        this.pos_x = 0.0;
      }
      if (initObj.hasOwnProperty('pos_y')) {
        this.pos_y = initObj.pos_y
      }
      else {
        this.pos_y = 0.0;
      }
      if (initObj.hasOwnProperty('pos_z')) {
        this.pos_z = initObj.pos_z
      }
      else {
        this.pos_z = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetPosRequest
    // Serialize message field [pos_x]
    bufferOffset = _serializer.float64(obj.pos_x, buffer, bufferOffset);
    // Serialize message field [pos_y]
    bufferOffset = _serializer.float64(obj.pos_y, buffer, bufferOffset);
    // Serialize message field [pos_z]
    bufferOffset = _serializer.float64(obj.pos_z, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetPosRequest
    let len;
    let data = new SetPosRequest(null);
    // Deserialize message field [pos_x]
    data.pos_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pos_y]
    data.pos_y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pos_z]
    data.pos_z = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a service object
    return 'plutodrone/SetPosRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'aa385529adbe642710281f4f0434423c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # SetPos.srv
    
    float64 pos_x
    float64 pos_y
    float64 pos_z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetPosRequest(null);
    if (msg.pos_x !== undefined) {
      resolved.pos_x = msg.pos_x;
    }
    else {
      resolved.pos_x = 0.0
    }

    if (msg.pos_y !== undefined) {
      resolved.pos_y = msg.pos_y;
    }
    else {
      resolved.pos_y = 0.0
    }

    if (msg.pos_z !== undefined) {
      resolved.pos_z = msg.pos_z;
    }
    else {
      resolved.pos_z = 0.0
    }

    return resolved;
    }
};

class SetPosResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.set_x = null;
      this.set_y = null;
      this.set_z = null;
    }
    else {
      if (initObj.hasOwnProperty('set_x')) {
        this.set_x = initObj.set_x
      }
      else {
        this.set_x = 0.0;
      }
      if (initObj.hasOwnProperty('set_y')) {
        this.set_y = initObj.set_y
      }
      else {
        this.set_y = 0.0;
      }
      if (initObj.hasOwnProperty('set_z')) {
        this.set_z = initObj.set_z
      }
      else {
        this.set_z = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetPosResponse
    // Serialize message field [set_x]
    bufferOffset = _serializer.float64(obj.set_x, buffer, bufferOffset);
    // Serialize message field [set_y]
    bufferOffset = _serializer.float64(obj.set_y, buffer, bufferOffset);
    // Serialize message field [set_z]
    bufferOffset = _serializer.float64(obj.set_z, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetPosResponse
    let len;
    let data = new SetPosResponse(null);
    // Deserialize message field [set_x]
    data.set_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [set_y]
    data.set_y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [set_z]
    data.set_z = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a service object
    return 'plutodrone/SetPosResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c5da85cbf47c5f9f317e3ca08de41a4e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 set_x
    float64 set_y
    float64 set_z
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetPosResponse(null);
    if (msg.set_x !== undefined) {
      resolved.set_x = msg.set_x;
    }
    else {
      resolved.set_x = 0.0
    }

    if (msg.set_y !== undefined) {
      resolved.set_y = msg.set_y;
    }
    else {
      resolved.set_y = 0.0
    }

    if (msg.set_z !== undefined) {
      resolved.set_z = msg.set_z;
    }
    else {
      resolved.set_z = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: SetPosRequest,
  Response: SetPosResponse,
  md5sum() { return '0887ae55fe5cc3ab69632861e1865920'; },
  datatype() { return 'plutodrone/SetPos'; }
};
