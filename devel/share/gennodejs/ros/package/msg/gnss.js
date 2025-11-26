// Auto-generated. Do not edit!

// (in-package package.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class gnss {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.East = null;
      this.North = null;
      this.Yaw = null;
    }
    else {
      if (initObj.hasOwnProperty('East')) {
        this.East = initObj.East
      }
      else {
        this.East = 0.0;
      }
      if (initObj.hasOwnProperty('North')) {
        this.North = initObj.North
      }
      else {
        this.North = 0.0;
      }
      if (initObj.hasOwnProperty('Yaw')) {
        this.Yaw = initObj.Yaw
      }
      else {
        this.Yaw = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type gnss
    // Serialize message field [East]
    bufferOffset = _serializer.float64(obj.East, buffer, bufferOffset);
    // Serialize message field [North]
    bufferOffset = _serializer.float64(obj.North, buffer, bufferOffset);
    // Serialize message field [Yaw]
    bufferOffset = _serializer.float64(obj.Yaw, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type gnss
    let len;
    let data = new gnss(null);
    // Deserialize message field [East]
    data.East = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [North]
    data.North = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Yaw]
    data.Yaw = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'package/gnss';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '786bb79528800c839905020a7bf7d1ae';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 East
    float64 North
    float64 Yaw
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new gnss(null);
    if (msg.East !== undefined) {
      resolved.East = msg.East;
    }
    else {
      resolved.East = 0.0
    }

    if (msg.North !== undefined) {
      resolved.North = msg.North;
    }
    else {
      resolved.North = 0.0
    }

    if (msg.Yaw !== undefined) {
      resolved.Yaw = msg.Yaw;
    }
    else {
      resolved.Yaw = 0.0
    }

    return resolved;
    }
};

module.exports = gnss;
