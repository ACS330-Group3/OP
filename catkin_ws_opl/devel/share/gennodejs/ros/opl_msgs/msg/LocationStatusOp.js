// Auto-generated. Do not edit!

// (in-package opl_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class LocationStatusOp {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.location = null;
      this.cube_at_ds = null;
      this.op_cube_id = null;
    }
    else {
      if (initObj.hasOwnProperty('location')) {
        this.location = initObj.location
      }
      else {
        this.location = '';
      }
      if (initObj.hasOwnProperty('cube_at_ds')) {
        this.cube_at_ds = initObj.cube_at_ds
      }
      else {
        this.cube_at_ds = false;
      }
      if (initObj.hasOwnProperty('op_cube_id')) {
        this.op_cube_id = initObj.op_cube_id
      }
      else {
        this.op_cube_id = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LocationStatusOp
    // Serialize message field [location]
    bufferOffset = _serializer.string(obj.location, buffer, bufferOffset);
    // Serialize message field [cube_at_ds]
    bufferOffset = _serializer.bool(obj.cube_at_ds, buffer, bufferOffset);
    // Serialize message field [op_cube_id]
    bufferOffset = _serializer.string(obj.op_cube_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LocationStatusOp
    let len;
    let data = new LocationStatusOp(null);
    // Deserialize message field [location]
    data.location = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [cube_at_ds]
    data.cube_at_ds = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [op_cube_id]
    data.op_cube_id = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.location.length;
    length += object.op_cube_id.length;
    return length + 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'opl_msgs/LocationStatusOp';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd7e59fda5792b384bfe9ef98e01f915a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string location
    bool cube_at_ds
    string op_cube_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new LocationStatusOp(null);
    if (msg.location !== undefined) {
      resolved.location = msg.location;
    }
    else {
      resolved.location = ''
    }

    if (msg.cube_at_ds !== undefined) {
      resolved.cube_at_ds = msg.cube_at_ds;
    }
    else {
      resolved.cube_at_ds = false
    }

    if (msg.op_cube_id !== undefined) {
      resolved.op_cube_id = msg.op_cube_id;
    }
    else {
      resolved.op_cube_id = ''
    }

    return resolved;
    }
};

module.exports = LocationStatusOp;
