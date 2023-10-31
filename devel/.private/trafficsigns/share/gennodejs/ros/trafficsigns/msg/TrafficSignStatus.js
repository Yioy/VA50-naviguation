// Auto-generated. Do not edit!

// (in-package trafficsigns.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let TrafficSign = require('./TrafficSign.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class TrafficSignStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.traffic_signs = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('traffic_signs')) {
        this.traffic_signs = initObj.traffic_signs
      }
      else {
        this.traffic_signs = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TrafficSignStatus
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [traffic_signs]
    // Serialize the length for message field [traffic_signs]
    bufferOffset = _serializer.uint32(obj.traffic_signs.length, buffer, bufferOffset);
    obj.traffic_signs.forEach((val) => {
      bufferOffset = TrafficSign.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TrafficSignStatus
    let len;
    let data = new TrafficSignStatus(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [traffic_signs]
    // Deserialize array length for message field [traffic_signs]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.traffic_signs = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.traffic_signs[i] = TrafficSign.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.traffic_signs.forEach((val) => {
      length += TrafficSign.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'trafficsigns/TrafficSignStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '481cb23ff8b604083652da43030d8361';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    TrafficSign[] traffic_signs
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: trafficsigns/TrafficSign
    string category
    string type
    float32 x
    float32 y
    float32 z
    float32 confidence
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TrafficSignStatus(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.traffic_signs !== undefined) {
      resolved.traffic_signs = new Array(msg.traffic_signs.length);
      for (let i = 0; i < resolved.traffic_signs.length; ++i) {
        resolved.traffic_signs[i] = TrafficSign.Resolve(msg.traffic_signs[i]);
      }
    }
    else {
      resolved.traffic_signs = []
    }

    return resolved;
    }
};

module.exports = TrafficSignStatus;
