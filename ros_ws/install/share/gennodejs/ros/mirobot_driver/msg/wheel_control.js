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

class wheel_control {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.dir_l = null;
      this.speed_l = null;
      this.dir_r = null;
      this.speed_r = null;
    }
    else {
      if (initObj.hasOwnProperty('dir_l')) {
        this.dir_l = initObj.dir_l
      }
      else {
        this.dir_l = 0;
      }
      if (initObj.hasOwnProperty('speed_l')) {
        this.speed_l = initObj.speed_l
      }
      else {
        this.speed_l = 0;
      }
      if (initObj.hasOwnProperty('dir_r')) {
        this.dir_r = initObj.dir_r
      }
      else {
        this.dir_r = 0;
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
    // Serializes a message object of type wheel_control
    // Serialize message field [dir_l]
    bufferOffset = _serializer.uint8(obj.dir_l, buffer, bufferOffset);
    // Serialize message field [speed_l]
    bufferOffset = _serializer.uint8(obj.speed_l, buffer, bufferOffset);
    // Serialize message field [dir_r]
    bufferOffset = _serializer.uint8(obj.dir_r, buffer, bufferOffset);
    // Serialize message field [speed_r]
    bufferOffset = _serializer.uint8(obj.speed_r, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type wheel_control
    let len;
    let data = new wheel_control(null);
    // Deserialize message field [dir_l]
    data.dir_l = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [speed_l]
    data.speed_l = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [dir_r]
    data.dir_r = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [speed_r]
    data.speed_r = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mirobot_driver/wheel_control';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7bf657bb437a3a998dcf9c0bf0cd51fa';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 dir_l
    uint8 speed_l
    uint8 dir_r
    uint8 speed_r
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new wheel_control(null);
    if (msg.dir_l !== undefined) {
      resolved.dir_l = msg.dir_l;
    }
    else {
      resolved.dir_l = 0
    }

    if (msg.speed_l !== undefined) {
      resolved.speed_l = msg.speed_l;
    }
    else {
      resolved.speed_l = 0
    }

    if (msg.dir_r !== undefined) {
      resolved.dir_r = msg.dir_r;
    }
    else {
      resolved.dir_r = 0
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

module.exports = wheel_control;
