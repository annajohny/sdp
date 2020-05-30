// Auto-generated. Do not edit!

// (in-package tug_observers_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class observation {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.observation_msg = null;
      this.verbose_observation_msg = null;
      this.observation = null;
    }
    else {
      if (initObj.hasOwnProperty('observation_msg')) {
        this.observation_msg = initObj.observation_msg
      }
      else {
        this.observation_msg = '';
      }
      if (initObj.hasOwnProperty('verbose_observation_msg')) {
        this.verbose_observation_msg = initObj.verbose_observation_msg
      }
      else {
        this.verbose_observation_msg = '';
      }
      if (initObj.hasOwnProperty('observation')) {
        this.observation = initObj.observation
      }
      else {
        this.observation = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type observation
    // Serialize message field [observation_msg]
    bufferOffset = _serializer.string(obj.observation_msg, buffer, bufferOffset);
    // Serialize message field [verbose_observation_msg]
    bufferOffset = _serializer.string(obj.verbose_observation_msg, buffer, bufferOffset);
    // Serialize message field [observation]
    bufferOffset = _serializer.int32(obj.observation, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type observation
    let len;
    let data = new observation(null);
    // Deserialize message field [observation_msg]
    data.observation_msg = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [verbose_observation_msg]
    data.verbose_observation_msg = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [observation]
    data.observation = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.observation_msg.length;
    length += object.verbose_observation_msg.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'tug_observers_msgs/observation';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '176310c5b8642d2cf705c70f2da7bc39';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new observation(null);
    if (msg.observation_msg !== undefined) {
      resolved.observation_msg = msg.observation_msg;
    }
    else {
      resolved.observation_msg = ''
    }

    if (msg.verbose_observation_msg !== undefined) {
      resolved.verbose_observation_msg = msg.verbose_observation_msg;
    }
    else {
      resolved.verbose_observation_msg = ''
    }

    if (msg.observation !== undefined) {
      resolved.observation = msg.observation;
    }
    else {
      resolved.observation = 0
    }

    return resolved;
    }
};

// Constants for message
observation.Constants = {
  GENERAL_OK: 0,
  GENERAL_ERROR: -1,
  NO_STATE_FITS: -2,
  NOT_AVAILABLE: -3,
  TIMEOUT: -4,
}

module.exports = observation;
