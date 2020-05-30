// Auto-generated. Do not edit!

// (in-package tug_resource_monitor.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class NodeInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.name = null;
      this.pid = null;
      this.hostname = null;
      this.cpu = null;
      this.memory = null;
      this.error = null;
    }
    else {
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = '';
      }
      if (initObj.hasOwnProperty('pid')) {
        this.pid = initObj.pid
      }
      else {
        this.pid = 0;
      }
      if (initObj.hasOwnProperty('hostname')) {
        this.hostname = initObj.hostname
      }
      else {
        this.hostname = '';
      }
      if (initObj.hasOwnProperty('cpu')) {
        this.cpu = initObj.cpu
      }
      else {
        this.cpu = 0.0;
      }
      if (initObj.hasOwnProperty('memory')) {
        this.memory = initObj.memory
      }
      else {
        this.memory = 0;
      }
      if (initObj.hasOwnProperty('error')) {
        this.error = initObj.error
      }
      else {
        this.error = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type NodeInfo
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    // Serialize message field [pid]
    bufferOffset = _serializer.uint32(obj.pid, buffer, bufferOffset);
    // Serialize message field [hostname]
    bufferOffset = _serializer.string(obj.hostname, buffer, bufferOffset);
    // Serialize message field [cpu]
    bufferOffset = _serializer.float32(obj.cpu, buffer, bufferOffset);
    // Serialize message field [memory]
    bufferOffset = _serializer.uint64(obj.memory, buffer, bufferOffset);
    // Serialize message field [error]
    bufferOffset = _serializer.uint32(obj.error, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type NodeInfo
    let len;
    let data = new NodeInfo(null);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [pid]
    data.pid = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [hostname]
    data.hostname = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [cpu]
    data.cpu = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [memory]
    data.memory = _deserializer.uint64(buffer, bufferOffset);
    // Deserialize message field [error]
    data.error = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.name.length;
    length += object.hostname.length;
    return length + 28;
  }

  static datatype() {
    // Returns string type for a message object
    return 'tug_resource_monitor/NodeInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b3df41a0cc3ca1f8f984ebc9825c7a08';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string name
    uint32 pid
    string hostname
    float32 cpu
    uint64 memory
    uint32 error
    uint32 NO_ERROR=0
    uint32 ERROR_PID_NOT_FOUND=1
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new NodeInfo(null);
    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = ''
    }

    if (msg.pid !== undefined) {
      resolved.pid = msg.pid;
    }
    else {
      resolved.pid = 0
    }

    if (msg.hostname !== undefined) {
      resolved.hostname = msg.hostname;
    }
    else {
      resolved.hostname = ''
    }

    if (msg.cpu !== undefined) {
      resolved.cpu = msg.cpu;
    }
    else {
      resolved.cpu = 0.0
    }

    if (msg.memory !== undefined) {
      resolved.memory = msg.memory;
    }
    else {
      resolved.memory = 0
    }

    if (msg.error !== undefined) {
      resolved.error = msg.error;
    }
    else {
      resolved.error = 0
    }

    return resolved;
    }
};

// Constants for message
NodeInfo.Constants = {
  NO_ERROR: 0,
  ERROR_PID_NOT_FOUND: 1,
}

module.exports = NodeInfo;
