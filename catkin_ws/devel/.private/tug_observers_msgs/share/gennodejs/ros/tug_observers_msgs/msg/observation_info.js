// Auto-generated. Do not edit!

// (in-package tug_observers_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let observation = require('./observation.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class observation_info {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.type = null;
      this.resource = null;
      this.observation = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('type')) {
        this.type = initObj.type
      }
      else {
        this.type = '';
      }
      if (initObj.hasOwnProperty('resource')) {
        this.resource = initObj.resource
      }
      else {
        this.resource = '';
      }
      if (initObj.hasOwnProperty('observation')) {
        this.observation = initObj.observation
      }
      else {
        this.observation = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type observation_info
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [type]
    bufferOffset = _serializer.string(obj.type, buffer, bufferOffset);
    // Serialize message field [resource]
    bufferOffset = _serializer.string(obj.resource, buffer, bufferOffset);
    // Serialize message field [observation]
    // Serialize the length for message field [observation]
    bufferOffset = _serializer.uint32(obj.observation.length, buffer, bufferOffset);
    obj.observation.forEach((val) => {
      bufferOffset = observation.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type observation_info
    let len;
    let data = new observation_info(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [type]
    data.type = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [resource]
    data.resource = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [observation]
    // Deserialize array length for message field [observation]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.observation = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.observation[i] = observation.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.type.length;
    length += object.resource.length;
    object.observation.forEach((val) => {
      length += observation.getMessageSize(val);
    });
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'tug_observers_msgs/observation_info';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '920b39a3d493095fc494ca757b21762f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    string type
    string resource
    observation[] observation
    
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
    # 0: no frame
    # 1: global frame
    string frame_id
    
    ================================================================================
    MSG: tug_observers_msgs/observation
    string observation_msg
    string verbose_observation_msg
    int32 observation
    int32 GENERAL_OK=0
    int32 GENERAL_ERROR=-1
    int32 NO_STATE_FITS=-2
    int32 NOT_AVAILABLE=-3
    int32 TIMEOUT=-4
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new observation_info(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.type !== undefined) {
      resolved.type = msg.type;
    }
    else {
      resolved.type = ''
    }

    if (msg.resource !== undefined) {
      resolved.resource = msg.resource;
    }
    else {
      resolved.resource = ''
    }

    if (msg.observation !== undefined) {
      resolved.observation = new Array(msg.observation.length);
      for (let i = 0; i < resolved.observation.length; ++i) {
        resolved.observation[i] = observation.Resolve(msg.observation[i]);
      }
    }
    else {
      resolved.observation = []
    }

    return resolved;
    }
};

module.exports = observation_info;
