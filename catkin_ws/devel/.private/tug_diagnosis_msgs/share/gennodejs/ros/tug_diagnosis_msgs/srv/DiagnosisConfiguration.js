// Auto-generated. Do not edit!

// (in-package tug_diagnosis_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let configuration = require('../msg/configuration.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class DiagnosisConfigurationRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.config = null;
      this.action = null;
    }
    else {
      if (initObj.hasOwnProperty('config')) {
        this.config = initObj.config
      }
      else {
        this.config = new configuration();
      }
      if (initObj.hasOwnProperty('action')) {
        this.action = initObj.action
      }
      else {
        this.action = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DiagnosisConfigurationRequest
    // Serialize message field [config]
    bufferOffset = configuration.serialize(obj.config, buffer, bufferOffset);
    // Serialize message field [action]
    bufferOffset = _serializer.int32(obj.action, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DiagnosisConfigurationRequest
    let len;
    let data = new DiagnosisConfigurationRequest(null);
    // Deserialize message field [config]
    data.config = configuration.deserialize(buffer, bufferOffset);
    // Deserialize message field [action]
    data.action = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += configuration.getMessageSize(object.config);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'tug_diagnosis_msgs/DiagnosisConfigurationRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd3d76205ac43f0b93af9b41d85fffda2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    configuration config
    
    
    int32 action
    int32 ADD=1
    int32 REMOVE=2
    int32 SET=3
    int32 UPDATE=4
    
    
    ================================================================================
    MSG: tug_diagnosis_msgs/configuration
    node_configuration[] nodes
    observer_configuration[] observers
    
    ================================================================================
    MSG: tug_diagnosis_msgs/node_configuration
    string name
    string[] sub_topic
    string[] pub_topic
    
    ================================================================================
    MSG: tug_diagnosis_msgs/observer_configuration
    string type
    string[] resource
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DiagnosisConfigurationRequest(null);
    if (msg.config !== undefined) {
      resolved.config = configuration.Resolve(msg.config)
    }
    else {
      resolved.config = new configuration()
    }

    if (msg.action !== undefined) {
      resolved.action = msg.action;
    }
    else {
      resolved.action = 0
    }

    return resolved;
    }
};

// Constants for message
DiagnosisConfigurationRequest.Constants = {
  ADD: 1,
  REMOVE: 2,
  SET: 3,
  UPDATE: 4,
}

class DiagnosisConfigurationResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.errorcode = null;
      this.error_msg = null;
    }
    else {
      if (initObj.hasOwnProperty('errorcode')) {
        this.errorcode = initObj.errorcode
      }
      else {
        this.errorcode = 0;
      }
      if (initObj.hasOwnProperty('error_msg')) {
        this.error_msg = initObj.error_msg
      }
      else {
        this.error_msg = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DiagnosisConfigurationResponse
    // Serialize message field [errorcode]
    bufferOffset = _serializer.int32(obj.errorcode, buffer, bufferOffset);
    // Serialize message field [error_msg]
    bufferOffset = _serializer.string(obj.error_msg, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DiagnosisConfigurationResponse
    let len;
    let data = new DiagnosisConfigurationResponse(null);
    // Deserialize message field [errorcode]
    data.errorcode = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [error_msg]
    data.error_msg = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.error_msg.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'tug_diagnosis_msgs/DiagnosisConfigurationResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '831df38a86121a0898c425468fa0cc9d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    int32 errorcode
    string error_msg
    int32 NO_ERROR=0
    int32 GENERAL_ERROR=-1
    int32 CONFIG_INVALID=-2
    
    
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DiagnosisConfigurationResponse(null);
    if (msg.errorcode !== undefined) {
      resolved.errorcode = msg.errorcode;
    }
    else {
      resolved.errorcode = 0
    }

    if (msg.error_msg !== undefined) {
      resolved.error_msg = msg.error_msg;
    }
    else {
      resolved.error_msg = ''
    }

    return resolved;
    }
};

// Constants for message
DiagnosisConfigurationResponse.Constants = {
  NO_ERROR: 0,
  GENERAL_ERROR: -1,
  CONFIG_INVALID: -2,
}

module.exports = {
  Request: DiagnosisConfigurationRequest,
  Response: DiagnosisConfigurationResponse,
  md5sum() { return '43fad42cb6cdc7f9a57a24dd1daa5334'; },
  datatype() { return 'tug_diagnosis_msgs/DiagnosisConfiguration'; }
};
