// Auto-generated. Do not edit!

// (in-package tug_observers_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let observation_info = require('./observation_info.js');

//-----------------------------------------------------------

class observer_info {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.observation_infos = null;
    }
    else {
      if (initObj.hasOwnProperty('observation_infos')) {
        this.observation_infos = initObj.observation_infos
      }
      else {
        this.observation_infos = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type observer_info
    // Serialize message field [observation_infos]
    // Serialize the length for message field [observation_infos]
    bufferOffset = _serializer.uint32(obj.observation_infos.length, buffer, bufferOffset);
    obj.observation_infos.forEach((val) => {
      bufferOffset = observation_info.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type observer_info
    let len;
    let data = new observer_info(null);
    // Deserialize message field [observation_infos]
    // Deserialize array length for message field [observation_infos]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.observation_infos = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.observation_infos[i] = observation_info.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.observation_infos.forEach((val) => {
      length += observation_info.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'tug_observers_msgs/observer_info';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '38c26f4d3dc2b7fc8f36eef35fb2083c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    observation_info[] observation_infos
    
    ================================================================================
    MSG: tug_observers_msgs/observation_info
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
    const resolved = new observer_info(null);
    if (msg.observation_infos !== undefined) {
      resolved.observation_infos = new Array(msg.observation_infos.length);
      for (let i = 0; i < resolved.observation_infos.length; ++i) {
        resolved.observation_infos[i] = observation_info.Resolve(msg.observation_infos[i]);
      }
    }
    else {
      resolved.observation_infos = []
    }

    return resolved;
    }
};

module.exports = observer_info;
