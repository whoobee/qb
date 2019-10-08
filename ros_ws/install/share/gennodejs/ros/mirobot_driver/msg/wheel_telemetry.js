// Auto-generated. Do not edit!

// (in-package mirobot_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class wheel_telemetry {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.speed_l = null;
      this.speed_r = null;
    }
    else {
      if (initObj.hasOwnProperty('speed_l')) {
        this.speed_l = initObj.speed_l
      }
      else {
        this.speed_l = 0;
      }
      if (initObj.hasOwnProperty('speed_r')) {
        this.speed_r = initObj.speed_r
      }
      else {
        this.speed_r = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type wheel_telemetry
    // Serialize message field [speed_l]
    bufferOffset = _serializer.int16(obj.speed_l, buffer, bufferOffset);
    // Serialize message field [speed_r]
    bufferOffset = _serializer.int16(obj.speed_r, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type wheel_telemetry
    let len;
    let data = new wheel_telemetry(null);
    // Deserialize message field [speed_l]
    data.speed_l = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [speed_r]
    data.speed_r = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mirobot_driver/wheel_telemetry';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6a13c855fd502a6ed208724a11e8020c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int16 speed_l
    int16 speed_r
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new wheel_telemetry(null);
    if (msg.speed_l !== undefined) {
      resolved.speed_l = msg.speed_l;
    }
    else {
      resolved.speed_l = 0
    }

    if (msg.speed_r !== undefined) {
      resolved.speed_r = msg.speed_r;
    }
    else {
      resolved.speed_r = 0
    }

    return resolved;
    }
};

module.exports = wheel_telemetry;
