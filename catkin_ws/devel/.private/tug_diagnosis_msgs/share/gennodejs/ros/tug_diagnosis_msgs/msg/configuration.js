// Auto-generated. Do not edit!

// (in-package tug_diagnosis_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let node_configuration = require('./node_configuration.js');
let observer_configuration = require('./observer_configuration.js');

//-----------------------------------------------------------

class configuration {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.nodes = null;
      this.observers = null;
    }
    else {
      if (initObj.hasOwnProperty('nodes')) {
        this.nodes = initObj.nodes
      }
      else {
        this.nodes = [];
      }
      if (initObj.hasOwnProperty('observers')) {
        this.observers = initObj.observers
      }
      else {
        this.observers = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type configuration
    // Serialize message field [nodes]
    // Serialize the length for message field [nodes]
    bufferOffset = _serializer.uint32(obj.nodes.length, buffer, bufferOffset);
    obj.nodes.forEach((val) => {
      bufferOffset = node_configuration.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [observers]
    // Serialize the length for message field [observers]
    bufferOffset = _serializer.uint32(obj.observers.length, buffer, bufferOffset);
    obj.observers.forEach((val) => {
      bufferOffset = observer_configuration.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type configuration
    let len;
    let data = new configuration(null);
    // Deserialize message field [nodes]
    // Deserialize array length for message field [nodes]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.nodes = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.nodes[i] = node_configuration.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [observers]
    // Deserialize array length for message field [observers]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.observers = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.observers[i] = observer_configuration.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.nodes.forEach((val) => {
      length += node_configuration.getMessageSize(val);
    });
    object.observers.forEach((val) => {
      length += observer_configuration.getMessageSize(val);
    });
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'tug_diagnosis_msgs/configuration';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cadde33865d5cf9cef30c348d588d747';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new configuration(null);
    if (msg.nodes !== undefined) {
      resolved.nodes = new Array(msg.nodes.length);
      for (let i = 0; i < resolved.nodes.length; ++i) {
        resolved.nodes[i] = node_configuration.Resolve(msg.nodes[i]);
      }
    }
    else {
      resolved.nodes = []
    }

    if (msg.observers !== undefined) {
      resolved.observers = new Array(msg.observers.length);
      for (let i = 0; i < resolved.observers.length; ++i) {
        resolved.observers[i] = observer_configuration.Resolve(msg.observers[i]);
      }
    }
    else {
      resolved.observers = []
    }

    return resolved;
    }
};

module.exports = configuration;
