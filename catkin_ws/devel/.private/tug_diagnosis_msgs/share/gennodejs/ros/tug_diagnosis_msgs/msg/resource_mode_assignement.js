// Auto-generated. Do not edit!

// (in-package tug_diagnosis_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class resource_mode_assignement {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.resource = null;
      this.mode_msg = null;
      this.mode = null;
    }
    else {
      if (initObj.hasOwnProperty('resource')) {
        this.resource = initObj.resource
      }
      else {
        this.resource = '';
      }
      if (initObj.hasOwnProperty('mode_msg')) {
        this.mode_msg = initObj.mode_msg
      }
      else {
        this.mode_msg = '';
      }
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type resource_mode_assignement
    // Serialize message field [resource]
    bufferOffset = _serializer.string(obj.resource, buffer, bufferOffset);
    // Serialize message field [mode_msg]
    bufferOffset = _serializer.string(obj.mode_msg, buffer, bufferOffset);
    // Serialize message field [mode]
    bufferOffset = _serializer.int32(obj.mode, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type resource_mode_assignement
    let len;
    let data = new resource_mode_assignement(null);
    // Deserialize message field [resource]
    data.resource = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [mode_msg]
    data.mode_msg = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [mode]
    data.mode = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.resource.length;
    length += object.mode_msg.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'tug_diagnosis_msgs/resource_mode_assignement';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3bb0fd0cf9c222fef578a66ea18f38c0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new resource_mode_assignement(null);
    if (msg.resource !== undefined) {
      resolved.resource = msg.resource;
    }
    else {
      resolved.resource = ''
    }

    if (msg.mode_msg !== undefined) {
      resolved.mode_msg = msg.mode_msg;
    }
    else {
      resolved.mode_msg = ''
    }

    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = 0
    }

    return resolved;
    }
};

// Constants for message
resource_mode_assignement.Constants = {
  GENERAL_OK: 0,
  GENERAL_ERROR: -1,
}

module.exports = resource_mode_assignement;
