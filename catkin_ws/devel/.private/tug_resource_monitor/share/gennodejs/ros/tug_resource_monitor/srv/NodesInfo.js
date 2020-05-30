// Auto-generated. Do not edit!

// (in-package tug_resource_monitor.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class NodesInfoRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.node_names = null;
    }
    else {
      if (initObj.hasOwnProperty('node_names')) {
        this.node_names = initObj.node_names
      }
      else {
        this.node_names = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type NodesInfoRequest
    // Serialize message field [node_names]
    bufferOffset = _arraySerializer.string(obj.node_names, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type NodesInfoRequest
    let len;
    let data = new NodesInfoRequest(null);
    // Deserialize message field [node_names]
    data.node_names = _arrayDeserializer.string(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.node_names.forEach((val) => {
      length += 4 + val.length;
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'tug_resource_monitor/NodesInfoRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '42c13fd17030d24481c0409739ac5ae7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string[] node_names
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new NodesInfoRequest(null);
    if (msg.node_names !== undefined) {
      resolved.node_names = msg.node_names;
    }
    else {
      resolved.node_names = []
    }

    return resolved;
    }
};

class NodesInfoResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type NodesInfoResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type NodesInfoResponse
    let len;
    let data = new NodesInfoResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'tug_resource_monitor/NodesInfoResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new NodesInfoResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: NodesInfoRequest,
  Response: NodesInfoResponse,
  md5sum() { return '42c13fd17030d24481c0409739ac5ae7'; },
  datatype() { return 'tug_resource_monitor/NodesInfo'; }
};
