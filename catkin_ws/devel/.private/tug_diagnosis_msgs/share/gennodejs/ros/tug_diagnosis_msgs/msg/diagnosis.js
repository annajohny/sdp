// Auto-generated. Do not edit!

// (in-package tug_diagnosis_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let resource_mode_assignement = require('./resource_mode_assignement.js');

//-----------------------------------------------------------

class diagnosis {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.diagnosis = null;
    }
    else {
      if (initObj.hasOwnProperty('diagnosis')) {
        this.diagnosis = initObj.diagnosis
      }
      else {
        this.diagnosis = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type diagnosis
    // Serialize message field [diagnosis]
    // Serialize the length for message field [diagnosis]
    bufferOffset = _serializer.uint32(obj.diagnosis.length, buffer, bufferOffset);
    obj.diagnosis.forEach((val) => {
      bufferOffset = resource_mode_assignement.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type diagnosis
    let len;
    let data = new diagnosis(null);
    // Deserialize message field [diagnosis]
    // Deserialize array length for message field [diagnosis]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.diagnosis = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.diagnosis[i] = resource_mode_assignement.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.diagnosis.forEach((val) => {
      length += resource_mode_assignement.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'tug_diagnosis_msgs/diagnosis';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '52161f8a0d1740c6fc459d4f815e3e6d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new diagnosis(null);
    if (msg.diagnosis !== undefined) {
      resolved.diagnosis = new Array(msg.diagnosis.length);
      for (let i = 0; i < resolved.diagnosis.length; ++i) {
        resolved.diagnosis[i] = resource_mode_assignement.Resolve(msg.diagnosis[i]);
      }
    }
    else {
      resolved.diagnosis = []
    }

    return resolved;
    }
};

module.exports = diagnosis;
