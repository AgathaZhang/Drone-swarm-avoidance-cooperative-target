#pragma once
// MESSAGE auto_filling_dance PACKING

#define MAVLINK_MSG_ID_auto_filling_dance 196


typedef struct __mavlink_auto_filling_dance_t {
 float pos[3]; /*<  */
 float acc[3]; /*<  */
 uint16_t frame; /*<  */
 uint16_t drone_id; /*<  */
 uint8_t rgb[3]; /*<  */
 uint8_t res; /*<  */
 uint8_t reserved[3]; /*<  */
} mavlink_auto_filling_dance_t;

#define MAVLINK_MSG_ID_auto_filling_dance_LEN 35
#define MAVLINK_MSG_ID_auto_filling_dance_MIN_LEN 35
#define MAVLINK_MSG_ID_196_LEN 35
#define MAVLINK_MSG_ID_196_MIN_LEN 35

#define MAVLINK_MSG_ID_auto_filling_dance_CRC 49
#define MAVLINK_MSG_ID_196_CRC 49

#define MAVLINK_MSG_auto_filling_dance_FIELD_POS_LEN 3
#define MAVLINK_MSG_auto_filling_dance_FIELD_ACC_LEN 3
#define MAVLINK_MSG_auto_filling_dance_FIELD_RGB_LEN 3
#define MAVLINK_MSG_auto_filling_dance_FIELD_RESERVED_LEN 3

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_auto_filling_dance { \
    196, \
    "auto_filling_dance", \
    7, \
    {  { "pos", NULL, MAVLINK_TYPE_FLOAT, 3, 0, offsetof(mavlink_auto_filling_dance_t, pos) }, \
         { "acc", NULL, MAVLINK_TYPE_FLOAT, 3, 12, offsetof(mavlink_auto_filling_dance_t, acc) }, \
         { "rgb", NULL, MAVLINK_TYPE_UINT8_T, 3, 28, offsetof(mavlink_auto_filling_dance_t, rgb) }, \
         { "frame", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_auto_filling_dance_t, frame) }, \
         { "drone_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 26, offsetof(mavlink_auto_filling_dance_t, drone_id) }, \
         { "res", NULL, MAVLINK_TYPE_UINT8_T, 0, 31, offsetof(mavlink_auto_filling_dance_t, res) }, \
         { "reserved", NULL, MAVLINK_TYPE_UINT8_T, 3, 32, offsetof(mavlink_auto_filling_dance_t, reserved) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_auto_filling_dance { \
    "auto_filling_dance", \
    7, \
    {  { "pos", NULL, MAVLINK_TYPE_FLOAT, 3, 0, offsetof(mavlink_auto_filling_dance_t, pos) }, \
         { "acc", NULL, MAVLINK_TYPE_FLOAT, 3, 12, offsetof(mavlink_auto_filling_dance_t, acc) }, \
         { "rgb", NULL, MAVLINK_TYPE_UINT8_T, 3, 28, offsetof(mavlink_auto_filling_dance_t, rgb) }, \
         { "frame", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_auto_filling_dance_t, frame) }, \
         { "drone_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 26, offsetof(mavlink_auto_filling_dance_t, drone_id) }, \
         { "res", NULL, MAVLINK_TYPE_UINT8_T, 0, 31, offsetof(mavlink_auto_filling_dance_t, res) }, \
         { "reserved", NULL, MAVLINK_TYPE_UINT8_T, 3, 32, offsetof(mavlink_auto_filling_dance_t, reserved) }, \
         } \
}
#endif

/**
 * @brief Pack a auto_filling_dance message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param pos  
 * @param acc  
 * @param rgb  
 * @param frame  
 * @param drone_id  
 * @param res  
 * @param reserved  
 * @return length of the message in bytes (excluding serial stream start sign)
 */

static inline uint16_t mavlink_msg_auto_filling_dance_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const float *pos, const float *acc, const uint8_t *rgb, uint16_t frame, uint16_t drone_id, uint8_t res, const uint8_t *reserved)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_auto_filling_dance_LEN];
    _mav_put_uint16_t(buf, 24, frame);
    _mav_put_uint16_t(buf, 26, drone_id);
    _mav_put_uint8_t(buf, 31, res);
    _mav_put_float_array(buf, 0, pos, 3);
    _mav_put_float_array(buf, 12, acc, 3);
    _mav_put_uint8_t_array(buf, 28, rgb, 3);
    _mav_put_uint8_t_array(buf, 32, reserved, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_auto_filling_dance_LEN);
#else
    mavlink_auto_filling_dance_t packet;
    packet.frame = frame;
    packet.drone_id = drone_id;
    packet.res = res;
    mav_array_memcpy(packet.pos, pos, sizeof(float)*3);
    mav_array_memcpy(packet.acc, acc, sizeof(float)*3);
    mav_array_memcpy(packet.rgb, rgb, sizeof(uint8_t)*3);
    mav_array_memcpy(packet.reserved, reserved, sizeof(uint8_t)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_auto_filling_dance_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_auto_filling_dance;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_auto_filling_dance_MIN_LEN, MAVLINK_MSG_ID_auto_filling_dance_LEN, MAVLINK_MSG_ID_auto_filling_dance_CRC);
}

/** 打包添加补位自定义数据格式2024.07.30*/  // TODO 改为mavlink_msg_auto_filling_dance_pack改为XML自动生成代码中的打包函数
static inline uint16_t mavlink_msg_bwcode_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_auto_filling_dance_t* qrcode)
{
	return mavlink_msg_auto_filling_dance_pack(system_id, component_id, msg, qrcode->pos, qrcode->acc, qrcode->rgb, qrcode->frame, qrcode->drone_id, qrcode->res, qrcode->reserved);
}

/**
 * @brief Pack a auto_filling_dance message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pos  
 * @param acc  
 * @param rgb  
 * @param frame  
 * @param drone_id  
 * @param res  
 * @param reserved  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_auto_filling_dance_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const float *pos,const float *acc,const uint8_t *rgb,uint16_t frame,uint16_t drone_id,uint8_t res,const uint8_t *reserved)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_auto_filling_dance_LEN];
    _mav_put_uint16_t(buf, 24, frame);
    _mav_put_uint16_t(buf, 26, drone_id);
    _mav_put_uint8_t(buf, 31, res);
    _mav_put_float_array(buf, 0, pos, 3);
    _mav_put_float_array(buf, 12, acc, 3);
    _mav_put_uint8_t_array(buf, 28, rgb, 3);
    _mav_put_uint8_t_array(buf, 32, reserved, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_auto_filling_dance_LEN);
#else
    mavlink_auto_filling_dance_t packet;
    packet.frame = frame;
    packet.drone_id = drone_id;
    packet.res = res;
    mav_array_memcpy(packet.pos, pos, sizeof(float)*3);
    mav_array_memcpy(packet.acc, acc, sizeof(float)*3);
    mav_array_memcpy(packet.rgb, rgb, sizeof(uint8_t)*3);
    mav_array_memcpy(packet.reserved, reserved, sizeof(uint8_t)*3);
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
    return mavlink_msg_auto_filling_dance_pack(system_id, component_id, msg, auto_filling_dance->pos, auto_filling_dance->acc, auto_filling_dance->rgb, auto_filling_dance->frame, auto_filling_dance->drone_id, auto_filling_dance->res, auto_filling_dance->reserved);
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
    return mavlink_msg_auto_filling_dance_pack_chan(system_id, component_id, chan, msg, auto_filling_dance->pos, auto_filling_dance->acc, auto_filling_dance->rgb, auto_filling_dance->frame, auto_filling_dance->drone_id, auto_filling_dance->res, auto_filling_dance->reserved);
}

/**
 * @brief Send a auto_filling_dance message
 * @param chan MAVLink channel to send the message
 *
 * @param pos  
 * @param acc  
 * @param rgb  
 * @param frame  
 * @param drone_id  
 * @param res  
 * @param reserved  
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_auto_filling_dance_send(mavlink_channel_t chan, const float *pos, const float *acc, const uint8_t *rgb, uint16_t frame, uint16_t drone_id, uint8_t res, const uint8_t *reserved)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_auto_filling_dance_LEN];
    _mav_put_uint16_t(buf, 24, frame);
    _mav_put_uint16_t(buf, 26, drone_id);
    _mav_put_uint8_t(buf, 31, res);
    _mav_put_float_array(buf, 0, pos, 3);
    _mav_put_float_array(buf, 12, acc, 3);
    _mav_put_uint8_t_array(buf, 28, rgb, 3);
    _mav_put_uint8_t_array(buf, 32, reserved, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_auto_filling_dance, buf, MAVLINK_MSG_ID_auto_filling_dance_MIN_LEN, MAVLINK_MSG_ID_auto_filling_dance_LEN, MAVLINK_MSG_ID_auto_filling_dance_CRC);
#else
    mavlink_auto_filling_dance_t packet;
    packet.frame = frame;
    packet.drone_id = drone_id;
    packet.res = res;
    mav_array_memcpy(packet.pos, pos, sizeof(float)*3);
    mav_array_memcpy(packet.acc, acc, sizeof(float)*3);
    mav_array_memcpy(packet.rgb, rgb, sizeof(uint8_t)*3);
    mav_array_memcpy(packet.reserved, reserved, sizeof(uint8_t)*3);
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
    mavlink_msg_auto_filling_dance_send(chan, auto_filling_dance->pos, auto_filling_dance->acc, auto_filling_dance->rgb, auto_filling_dance->frame, auto_filling_dance->drone_id, auto_filling_dance->res, auto_filling_dance->reserved);
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
static inline void mavlink_msg_auto_filling_dance_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const float *pos, const float *acc, const uint8_t *rgb, uint16_t frame, uint16_t drone_id, uint8_t res, const uint8_t *reserved)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 24, frame);
    _mav_put_uint16_t(buf, 26, drone_id);
    _mav_put_uint8_t(buf, 31, res);
    _mav_put_float_array(buf, 0, pos, 3);
    _mav_put_float_array(buf, 12, acc, 3);
    _mav_put_uint8_t_array(buf, 28, rgb, 3);
    _mav_put_uint8_t_array(buf, 32, reserved, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_auto_filling_dance, buf, MAVLINK_MSG_ID_auto_filling_dance_MIN_LEN, MAVLINK_MSG_ID_auto_filling_dance_LEN, MAVLINK_MSG_ID_auto_filling_dance_CRC);
#else
    mavlink_auto_filling_dance_t *packet = (mavlink_auto_filling_dance_t *)msgbuf;
    packet->frame = frame;
    packet->drone_id = drone_id;
    packet->res = res;
    mav_array_memcpy(packet->pos, pos, sizeof(float)*3);
    mav_array_memcpy(packet->acc, acc, sizeof(float)*3);
    mav_array_memcpy(packet->rgb, rgb, sizeof(uint8_t)*3);
    mav_array_memcpy(packet->reserved, reserved, sizeof(uint8_t)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_auto_filling_dance, (const char *)packet, MAVLINK_MSG_ID_auto_filling_dance_MIN_LEN, MAVLINK_MSG_ID_auto_filling_dance_LEN, MAVLINK_MSG_ID_auto_filling_dance_CRC);
#endif
}
#endif

#endif

// MESSAGE auto_filling_dance UNPACKING


/**
 * @brief Get field pos from auto_filling_dance message
 *
 * @return  
 */
static inline uint16_t mavlink_msg_auto_filling_dance_get_pos(const mavlink_message_t* msg, float *pos)
{
    return _MAV_RETURN_float_array(msg, pos, 3,  0);
}

/**
 * @brief Get field acc from auto_filling_dance message
 *
 * @return  
 */
static inline uint16_t mavlink_msg_auto_filling_dance_get_acc(const mavlink_message_t* msg, float *acc)
{
    return _MAV_RETURN_float_array(msg, acc, 3,  12);
}

/**
 * @brief Get field rgb from auto_filling_dance message
 *
 * @return  
 */
static inline uint16_t mavlink_msg_auto_filling_dance_get_rgb(const mavlink_message_t* msg, uint8_t *rgb)
{
    return _MAV_RETURN_uint8_t_array(msg, rgb, 3,  28);
}

/**
 * @brief Get field frame from auto_filling_dance message
 *
 * @return  
 */
static inline uint16_t mavlink_msg_auto_filling_dance_get_frame(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  24);
}

/**
 * @brief Get field drone_id from auto_filling_dance message
 *
 * @return  
 */
static inline uint16_t mavlink_msg_auto_filling_dance_get_drone_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  26);
}

/**
 * @brief Get field res from auto_filling_dance message
 *
 * @return  
 */
static inline uint8_t mavlink_msg_auto_filling_dance_get_res(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  31);
}

/**
 * @brief Get field reserved from auto_filling_dance message
 *
 * @return  
 */
static inline uint16_t mavlink_msg_auto_filling_dance_get_reserved(const mavlink_message_t* msg, uint8_t *reserved)
{
    return _MAV_RETURN_uint8_t_array(msg, reserved, 3,  32);
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
    mavlink_msg_auto_filling_dance_get_pos(msg, auto_filling_dance->pos);
    mavlink_msg_auto_filling_dance_get_acc(msg, auto_filling_dance->acc);
    auto_filling_dance->frame = mavlink_msg_auto_filling_dance_get_frame(msg);
    auto_filling_dance->drone_id = mavlink_msg_auto_filling_dance_get_drone_id(msg);
    mavlink_msg_auto_filling_dance_get_rgb(msg, auto_filling_dance->rgb);
    auto_filling_dance->res = mavlink_msg_auto_filling_dance_get_res(msg);
    mavlink_msg_auto_filling_dance_get_reserved(msg, auto_filling_dance->reserved);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_auto_filling_dance_LEN? msg->len : MAVLINK_MSG_ID_auto_filling_dance_LEN;
        memset(auto_filling_dance, 0, MAVLINK_MSG_ID_auto_filling_dance_LEN);
    memcpy(auto_filling_dance, _MAV_PAYLOAD(msg), len);
#endif
}
