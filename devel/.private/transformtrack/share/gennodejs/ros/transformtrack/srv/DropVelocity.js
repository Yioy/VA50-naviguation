// Auto-generated. Do not edit!

// (in-package transformtrack.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class DropVelocityRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.end_time = null;
      this.unbias = null;
    }
    else {
      if (initObj.hasOwnProperty('end_time')) {
        this.end_time = initObj.end_time
      }
      else {
        this.end_time = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('unbias')) {
        this.unbias = initObj.unbias
      }
      else {
        this.unbias = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DropVelocityRequest
    // Serialize message field [end_time]
    bufferOffset = _serializer.time(obj.end_time, buffer, bufferOffset);
    // Serialize message field [unbias]
    bufferOffset = _serializer.bool(obj.unbias, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DropVelocityRequest
    let len;
    let data = new DropVelocityRequest(null);
    // Deserialize message field [end_time]
    data.end_time = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [unbias]
    data.unbias = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 9;
  }

  static datatype() {
    // Returns string type for a service object
    return 'transformtrack/DropVelocityRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'faa0b1c70541a1016cc8927509fb3480';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    time end_time
    bool unbias
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DropVelocityRequest(null);
    if (msg.end_time !== undefined) {
      resolved.end_time = msg.end_time;
    }
    else {
      resolved.end_time = {secs: 0, nsecs: 0}
    }

    if (msg.unbias !== undefined) {
      resolved.unbias = msg.unbias;
    }
    else {
      resolved.unbias = false
    }

    return resolved;
    }
};

class DropVelocityResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.done = null;
    }
    else {
      if (initObj.hasOwnProperty('done')) {
        this.done = initObj.done
      }
      else {
        this.done = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DropVelocityResponse
    // Serialize message field [done]
    bufferOffset = _serializer.bool(obj.done, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DropVelocityResponse
    let len;
    let data = new DropVelocityResponse(null);
    // Deserialize message field [done]
    data.done = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'transformtrack/DropVelocityResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '89bb254424e4cffedbf494e7b0ddbfea';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool done
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DropVelocityResponse(null);
    if (msg.done !== undefined) {
      resolved.done = msg.done;
    }
    else {
      resolved.done = false
    }

    return resolved;
    }
};

module.exports = {
  Request: DropVelocityRequest,
  Response: DropVelocityResponse,
  md5sum() { return '2b47bafdc4c00e27a7f0cfd76dc9037e'; },
  datatype() { return 'transformtrack/DropVelocity'; }
};
