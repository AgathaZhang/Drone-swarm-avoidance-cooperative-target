#pragma once
// MESSAGE auto_filling_dance PACKING

#define MAVLINK_MSG_ID_auto_filling_dance 196


typedef struct __mavlink_auto_filling_dance_t {
 float x; /*<  */
 float y; /*<  */
 float z; /*<  */
 uint32_t frame; /*<  */
 uint8_t reserved[5]; /*<  */
} mavlink_auto_filling_dance_t;

#define MAVLINK_MSG_ID_auto_filling_dance_LEN 21
#define MAVLINK_MSG_ID_auto_filling_dance_MIN_LEN 21
#define MAVLINK_MSG_ID_196_LEN 21
#define MAVLINK_MSG_ID_196_MIN_LEN 21

#define MAVLINK_MSG_ID_auto_filling_dance_CRC 126
#define MAVLINK_MSG_ID_196_CRC 126

#define MAVLINK_MSG_auto_filling_dance_FIELD_RESERVED_LEN 5

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_auto_filling_dance { \
    196, \
    "auto_filling_dance", \
    5, \
    {  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_auto_filling_dance_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_auto_filling_dance_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_auto_filling_dance_t, z) }, \
         { "frame", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_auto_filling_dance_t, frame) }, \
         { "reserved", NULL, MAVLINK_TYPE_UINT8_T, 5, 16, offsetof(mavlink_auto_filling_dance_t, reserved) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_auto_filling_dance { \
    "auto_filling_dance", \
    5, \
    {  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_auto_filling_dance_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_auto_filling_dance_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_auto_filling_dance_t, z) }, \
         { "frame", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_auto_filling_dance_t, frame) }, \
         { "reserved", NULL, MAVLINK_TYPE_UINT8_T, 5, 16, offsetof(mavlink_auto_filling_dance_t, reserved) }, \
         } \
}
#endif

/**
 * @brief Pack a auto_filling_dance message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param x  
 * @param y  
 * @param z  
 * @param frame  
 * @param reserved  
 * @return length of the message in bytes (excluding serial stream start sign)
 */

static inline uint16_t mavlink_msg_bwcode_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_auto_filling_dance_t* qrcode);  //添加提前申明

static inline uint16_t mavlink_msg_auto_filling_dance_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float x, float y, float z, uint32_t frame, const uint8_t *reserved)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_auto_filling_dance_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_uint32_t(buf, 12, frame);
    _mav_put_uint8_t_array(buf, 16, reserved, 5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_auto_filling_dance_LEN);
#else
    mavlink_auto_filling_dance_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.frame = frame;
    mav_array_memcpy(packet.reserved, reserved, sizeof(uint8_t)*5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_auto_filling_dance_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_auto_filling_dance;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_auto_filling_dance_MIN_LEN, MAVLINK_MSG_ID_auto_filling_dance_LEN, MAVLINK_MSG_ID_auto_filling_dance_CRC);
}

/** 打包添加补位自定义数据格式2024.07.30*/  // TODO 改为mavlink_msg_auto_filling_dance_pack改为XML自动生成代码中的打包函数
static inline uint16_t mavlink_msg_bwcode_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_auto_filling_dance_t* qrcode)
{
	return mavlink_msg_auto_filling_dance_pack(system_id, component_id, msg, qrcode->x,qrcode->y, qrcode->z, qrcode->frame, qrcode->reserved);
}

/**
 * @brief Pack a auto_filling_dance message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param x  
 * @param y  
 * @param z  
 * @param frame  
 * @param reserved  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_auto_filling_dance_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float x,float y,float z,uint32_t frame,const uint8_t *reserved)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_auto_filling_dance_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_uint32_t(buf, 12, frame);
    _mav_put_uint8_t_array(buf, 16, reserved, 5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_auto_filling_dance_LEN);
#else
    mavlink_auto_filling_dance_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.frame = frame;
    mav_array_memcpy(packet.reserved, reserved, sizeof(uint8_t)*5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_auto_filling_dance_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_auto_filling_dance;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_auto_filling_dance_MIN_LEN, MAVLINK_MSG_ID_auto_filling_dance_LEN, MAVLINK_MSG_ID_auto_filling_dance_CRC);
}

/**
 * @brief Encode a auto_filling_dance struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param auto_filling_dance C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_auto_filling_dance_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_auto_filling_dance_t* auto_filling_dance)
{
    return mavlink_msg_auto_filling_dance_pack(system_id, component_id, msg, auto_filling_dance->x, auto_filling_dance->y, auto_filling_dance->z, auto_filling_dance->frame, auto_filling_dance->reserved);
}

/**
 * @brief Encode a auto_filling_dance struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param auto_filling_dance C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_auto_filling_dance_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_auto_filling_dance_t* auto_filling_dance)
{
    return mavlink_msg_auto_filling_dance_pack_chan(system_id, component_id, chan, msg, auto_filling_dance->x, auto_filling_dance->y, auto_filling_dance->z, auto_filling_dance->frame, auto_filling_dance->reserved);
}

/**
 * @brief Send a auto_filling_dance message
 * @param chan MAVLink channel to send the message
 *
 * @param x  
 * @param y  
 * @param z  
 * @param frame  
 * @param reserved  
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_auto_filling_dance_send(mavlink_channel_t chan, float x, float y, float z, uint32_t frame, const uint8_t *reserved)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_auto_filling_dance_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_uint32_t(buf, 12, frame);
    _mav_put_uint8_t_array(buf, 16, reserved, 5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_auto_filling_dance, buf, MAVLINK_MSG_ID_auto_filling_dance_MIN_LEN, MAVLINK_MSG_ID_auto_filling_dance_LEN, MAVLINK_MSG_ID_auto_filling_dance_CRC);
#else
    mavlink_auto_filling_dance_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.frame = frame;
    mav_array_memcpy(packet.reserved, reserved, sizeof(uint8_t)*5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_auto_filling_dance, (const char *)&packet, MAVLINK_MSG_ID_auto_filling_dance_MIN_LEN, MAVLINK_MSG_ID_auto_filling_dance_LEN, MAVLINK_MSG_ID_auto_filling_dance_CRC);
#endif
}

/**
 * @brief Send a auto_filling_dance message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_auto_filling_dance_send_struct(mavlink_channel_t chan, const mavlink_auto_filling_dance_t* auto_filling_dance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_auto_filling_dance_send(chan, auto_filling_dance->x, auto_filling_dance->y, auto_filling_dance->z, auto_filling_dance->frame, auto_filling_dance->reserved);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_auto_filling_dance, (const char *)auto_filling_dance, MAVLINK_MSG_ID_auto_filling_dance_MIN_LEN, MAVLINK_MSG_ID_auto_filling_dance_LEN, MAVLINK_MSG_ID_auto_filling_dance_CRC);
#endif
}

#if MAVLINK_MSG_ID_auto_filling_dance_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_auto_filling_dance_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float x, float y, float z, uint32_t frame, const uint8_t *reserved)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_uint32_t(buf, 12, frame);
    _mav_put_uint8_t_array(buf, 16, reserved, 5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_auto_filling_dance, buf, MAVLINK_MSG_ID_auto_filling_dance_MIN_LEN, MAVLINK_MSG_ID_auto_filling_dance_LEN, MAVLINK_MSG_ID_auto_filling_dance_CRC);
#else
    mavlink_auto_filling_dance_t *packet = (mavlink_auto_filling_dance_t *)msgbuf;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->frame = frame;
    mav_array_memcpy(packet->reserved, reserved, sizeof(uint8_t)*5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_auto_filling_dance, (const char *)packet, MAVLINK_MSG_ID_auto_filling_dance_MIN_LEN, MAVLINK_MSG_ID_auto_filling_dance_LEN, MAVLINK_MSG_ID_auto_filling_dance_CRC);
#endif
}
#endif

#endif

// MESSAGE auto_filling_dance UNPACKING


/**
 * @brief Get field x from auto_filling_dance message
 *
 * @return  
 */
static inline float mavlink_msg_auto_filling_dance_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field y from auto_filling_dance message
 *
 * @return  
 */
static inline float mavlink_msg_auto_filling_dance_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field z from auto_filling_dance message
 *
 * @return  
 */
static inline float mavlink_msg_auto_filling_dance_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field frame from auto_filling_dance message
 *
 * @return  
 */
static inline uint32_t mavlink_msg_auto_filling_dance_get_frame(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Get field reserved from auto_filling_dance message
 *
 * @return  
 */
static inline uint16_t mavlink_msg_auto_filling_dance_get_reserved(const mavlink_message_t* msg, uint8_t *reserved)
{
    return _MAV_RETURN_uint8_t_array(msg, reserved, 5,  16);
}

/**
 * @brief Decode a auto_filling_dance message into a struct
 *
 * @param msg The message to decode
 * @param auto_filling_dance C-struct to decode the message contents into
 */
static inline void mavlink_msg_auto_filling_dance_decode(const mavlink_message_t* msg, mavlink_auto_filling_dance_t* auto_filling_dance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    auto_filling_dance->x = mavlink_msg_auto_filling_dance_get_x(msg);
    auto_filling_dance->y = mavlink_msg_auto_filling_dance_get_y(msg);
    auto_filling_dance->z = mavlink_msg_auto_filling_dance_get_z(msg);
    auto_filling_dance->frame = mavlink_msg_auto_filling_dance_get_frame(msg);
    mavlink_msg_auto_filling_dance_get_reserved(msg, auto_filling_dance->reserved);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_auto_filling_dance_LEN? msg->len : MAVLINK_MSG_ID_auto_filling_dance_LEN;
        memset(auto_filling_dance, 0, MAVLINK_MSG_ID_auto_filling_dance_LEN);
    memcpy(auto_filling_dance, _MAV_PAYLOAD(msg), len);
#endif
}
