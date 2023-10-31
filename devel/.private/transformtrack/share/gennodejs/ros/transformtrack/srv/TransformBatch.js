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

let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class TransformBatchRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.start_times = null;
      this.end_time = null;
    }
    else {
      if (initObj.hasOwnProperty('start_times')) {
        this.start_times = initObj.start_times
      }
      else {
        this.start_times = [];
      }
      if (initObj.hasOwnProperty('end_time')) {
        this.end_time = initObj.end_time
      }
      else {
        this.end_time = {secs: 0, nsecs: 0};
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TransformBatchRequest
    // Serialize message field [start_times]
    bufferOffset = _arraySerializer.time(obj.start_times, buffer, bufferOffset, null);
    // Serialize message field [end_time]
    bufferOffset = _serializer.time(obj.end_time, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TransformBatchRequest
    let len;
    let data = new TransformBatchRequest(null);
    // Deserialize message field [start_times]
    data.start_times = _arrayDeserializer.time(buffer, bufferOffset, null)
    // Deserialize message field [end_time]
    data.end_time = _deserializer.time(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.start_times.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'transformtrack/TransformBatchRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bdba198c1efa142b516a98e81cf09cee';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    time[] start_times
    time end_time
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TransformBatchRequest(null);
    if (msg.start_times !== undefined) {
      resolved.start_times = msg.start_times;
    }
    else {
      resolved.start_times = []
    }

    if (msg.end_time !== undefined) {
      resolved.end_time = msg.end_time;
    }
    else {
      resolved.end_time = {secs: 0, nsecs: 0}
    }

    return resolved;
    }
};

class TransformBatchResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.transforms = null;
      this.distances = null;
    }
    else {
      if (initObj.hasOwnProperty('transforms')) {
        this.transforms = initObj.transforms
      }
      else {
        this.transforms = new std_msgs.msg.Float64MultiArray();
      }
      if (initObj.hasOwnProperty('distances')) {
        this.distances = initObj.distances
      }
      else {
        this.distances = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TransformBatchResponse
    // Serialize message field [transforms]
    bufferOffset = std_msgs.msg.Float64MultiArray.serialize(obj.transforms, buffer, bufferOffset);
    // Serialize message field [distances]
    bufferOffset = _arraySerializer.float64(obj.distances, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TransformBatchResponse
    let len;
    let data = new TransformBatchResponse(null);
    // Deserialize message field [transforms]
    data.transforms = std_msgs.msg.Float64MultiArray.deserialize(buffer, bufferOffset);
    // Deserialize message field [distances]
    data.distances = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Float64MultiArray.getMessageSize(object.transforms);
    length += 8 * object.distances.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'transformtrack/TransformBatchResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ca8be60547a73646ae9c3cf9156c7ab7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Float64MultiArray transforms
    float64[] distances
    
    ================================================================================
    MSG: std_msgs/Float64MultiArray
    # Please look at the MultiArrayLayout message definition for
    # documentation on all multiarrays.
    
    MultiArrayLayout  layout        # specification of data layout
    float64[]         data          # array of data
    
    
    ================================================================================
    MSG: std_msgs/MultiArrayLayout
    # The multiarray declares a generic multi-dimensional array of a
    # particular data type.  Dimensions are ordered from outer most
    # to inner most.
    
    MultiArrayDimension[] dim # Array of dimension properties
    uint32 data_offset        # padding elements at front of data
    
    # Accessors should ALWAYS be written in terms of dimension stride
    # and specified outer-most dimension first.
    # 
    # multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
    #
    # A standard, 3-channel 640x480 image with interleaved color channels
    # would be specified as:
    #
    # dim[0].label  = "height"
    # dim[0].size   = 480
    # dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
    # dim[1].label  = "width"
    # dim[1].size   = 640
    # dim[1].stride = 3*640 = 1920
    # dim[2].label  = "channel"
    # dim[2].size   = 3
    # dim[2].stride = 3
    #
    # multiarray(i,j,k) refers to the ith row, jth column, and kth channel.
    
    ================================================================================
    MSG: std_msgs/MultiArrayDimension
    string label   # label of given dimension
    uint32 size    # size of given dimension (in type units)
    uint32 stride  # stride of given dimension
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TransformBatchResponse(null);
    if (msg.transforms !== undefined) {
      resolved.transforms = std_msgs.msg.Float64MultiArray.Resolve(msg.transforms)
    }
    else {
      resolved.transforms = new std_msgs.msg.Float64MultiArray()
    }

    if (msg.distances !== undefined) {
      resolved.distances = msg.distances;
    }
    else {
      resolved.distances = []
    }

    return resolved;
    }
};

module.exports = {
  Request: TransformBatchRequest,
  Response: TransformBatchResponse,
  md5sum() { return 'd17c54b06f263044d75d6d03617a9ba1'; },
  datatype() { return 'transformtrack/TransformBatch'; }
};
