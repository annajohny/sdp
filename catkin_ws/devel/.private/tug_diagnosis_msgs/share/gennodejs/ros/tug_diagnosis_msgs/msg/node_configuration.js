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

class node_configuration {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.name = null;
      this.sub_topic = null;
      this.pub_topic = null;
    }
    else {
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = '';
      }
      if (initObj.hasOwnProperty('sub_topic')) {
        this.sub_topic = initObj.sub_topic
      }
      else {
        this.sub_topic = [];
      }
      if (initObj.hasOwnProperty('pub_topic')) {
        this.pub_topic = initObj.pub_topic
      }
      else {
        this.pub_topic = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type node_configuration
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    // Serialize message field [sub_topic]
    bufferOffset = _arraySerializer.string(obj.sub_topic, buffer, bufferOffset, null);
    // Serialize message field [pub_topic]
    bufferOffset = _arraySerializer.string(obj.pub_topic, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type node_configuration
    let len;
    let data = new node_configuration(null);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [sub_topic]
    data.sub_topic = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [pub_topic]
    data.pub_topic = _arrayDeserializer.string(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.name.length;
    object.sub_topic.forEach((val) => {
      length += 4 + val.length;
    });
    object.pub_topic.forEach((val) => {
      length += 4 + val.length;
    });
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'tug_diagnosis_msgs/node_configuration';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd437a072fcff0973a4326ed765667e76';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string name
    string[] sub_topic
    string[] pub_topic
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new node_configuration(null);
    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = ''
    }

    if (msg.sub_topic !== undefined) {
      resolved.sub_topic = msg.sub_topic;
    }
    else {
      resolved.sub_topic = []
    }

    if (msg.pub_topic !== undefined) {
      resolved.pub_topic = msg.pub_topic;
    }
    else {
      resolved.pub_topic = []
    }

    return resolved;
    }
};

module.exports = node_configuration;
