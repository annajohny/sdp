// Auto-generated. Do not edit!

// (in-package tug_diagnosis_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let diagnosis = require('./diagnosis.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class diagnosis_set {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.type = null;
      this.diagnoses = null;
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
      if (initObj.hasOwnProperty('diagnoses')) {
        this.diagnoses = initObj.diagnoses
      }
      else {
        this.diagnoses = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type diagnosis_set
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [type]
    bufferOffset = _serializer.string(obj.type, buffer, bufferOffset);
    // Serialize message field [diagnoses]
    // Serialize the length for message field [diagnoses]
    bufferOffset = _serializer.uint32(obj.diagnoses.length, buffer, bufferOffset);
    obj.diagnoses.forEach((val) => {
      bufferOffset = diagnosis.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type diagnosis_set
    let len;
    let data = new diagnosis_set(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [type]
    data.type = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [diagnoses]
    // Deserialize array length for message field [diagnoses]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.diagnoses = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.diagnoses[i] = diagnosis.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.type.length;
    object.diagnoses.forEach((val) => {
      length += diagnosis.getMessageSize(val);
    });
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'tug_diagnosis_msgs/diagnosis_set';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1e8ee6a1d3f192b8969b6d2785e6690b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    string type
    diagnosis[] diagnoses
    
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
    MSG: tug_diagnosis_msgs/diagnosis
    resource_mode_assignement[] diagnosis
    
    ================================================================================
    MSG: tug_diagnosis_msgs/resource_mode_assignement
    string resource
    string mode_msg
    int32 mode
    int32 GENERAL_OK=0
    int32 GENERAL_ERROR=-1
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new diagnosis_set(null);
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

    if (msg.diagnoses !== undefined) {
      resolved.diagnoses = new Array(msg.diagnoses.length);
      for (let i = 0; i < resolved.diagnoses.length; ++i) {
        resolved.diagnoses[i] = diagnosis.Resolve(msg.diagnoses[i]);
      }
    }
    else {
      resolved.diagnoses = []
    }

    return resolved;
    }
};

module.exports = diagnosis_set;
