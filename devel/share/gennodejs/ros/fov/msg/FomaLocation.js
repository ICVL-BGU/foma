// Auto-generated. Do not edit!

// (in-package fov.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class FomaLocation {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.image = null;
      this.world = null;
    }
    else {
      if (initObj.hasOwnProperty('image')) {
        this.image = initObj.image
      }
      else {
        this.image = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('world')) {
        this.world = initObj.world
      }
      else {
        this.world = new geometry_msgs.msg.Point();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FomaLocation
    // Serialize message field [image]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.image, buffer, bufferOffset);
    // Serialize message field [world]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.world, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FomaLocation
    let len;
    let data = new FomaLocation(null);
    // Deserialize message field [image]
    data.image = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [world]
    data.world = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 48;
  }

  static datatype() {
    // Returns string type for a message object
    return 'fov/FomaLocation';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5b07c96efdecb7282bf61c35bd56a0d2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Point image
    geometry_msgs/Point world
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FomaLocation(null);
    if (msg.image !== undefined) {
      resolved.image = geometry_msgs.msg.Point.Resolve(msg.image)
    }
    else {
      resolved.image = new geometry_msgs.msg.Point()
    }

    if (msg.world !== undefined) {
      resolved.world = geometry_msgs.msg.Point.Resolve(msg.world)
    }
    else {
      resolved.world = new geometry_msgs.msg.Point()
    }

    return resolved;
    }
};

module.exports = FomaLocation;
