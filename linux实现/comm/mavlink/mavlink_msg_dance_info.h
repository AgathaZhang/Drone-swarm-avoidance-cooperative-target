#pragma once
// MESSAGE DANCE_INFO PACKING

#define MAVLINK_MSG_ID_DANCE_INFO 238


typedef struct __mavlink_dance_info_t {
 uint32_t dance_size; /*<  dance size*/
 uint8_t cmd; /*<  cmd*/
 char dance_name[64]; /*<  dance name*/
 uint8_t dance_md5[16]; /*<  dance md5*/
} mavlink_dance_info_t;

#define MAVLINK_MSG_ID_DANCE_INFO_LEN 85
#define MAVLINK_MSG_ID_DANCE_INFO_MIN_LEN 85
#define MAVLINK_MSG_ID_238_LEN 85
#define MAVLINK_MSG_ID_238_MIN_LEN 85

#define MAVLINK_MSG_ID_DANCE_INFO_CRC 88
#define MAVLINK_MSG_ID_238_CRC 88

#define MAVLINK_MSG_DANCE_INFO_FIELD_DANCE_NAME_LEN 64
#define MAVLINK_MSG_DANCE_INFO_FIELD_DANCE_MD5_LEN 16

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DANCE_INFO { \
    238, \
    "DANCE_INFO", \
    4, \
    {  { "cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_dance_info_t, cmd) }, \
         { "dance_name", NULL, MAVLINK_TYPE_CHAR, 64, 5, offsetof(mavlink_dance_info_t, dance_name) }, \
         { "dance_size", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_dance_info_t, dance_size) }, \
         { "dance_md5", NULL, MAVLINK_TYPE_UINT8_T, 16, 69, offsetof(mavlink_dance_info_t, dance_md5) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DANCE_INFO { \
    "DANCE_INFO", \
    4, \
    {  { "cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_dance_info_t, cmd) }, \
         { "dance_name", NULL, MAVLINK_TYPE_CHAR, 64, 5, offsetof(mavlink_dance_info_t, dance_name) }, \
         { "dance_size", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_dance_info_t, dance_size) }, \
         { "dance_md5", NULL, MAVLINK_TYPE_UINT8_T, 16, 69, offsetof(mavlink_dance_info_t, dance_md5) }, \
         } \
}
#endif

/**
 * @brief Pack a dance_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param cmd  cmd
 * @param dance_name  dance name
 * @param dance_size  dance size
 * @param dance_md5  dance md5
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_dance_info_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t cmd, const char *dance_name, uint32_t dance_size, const uint8_t *dance_md5)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DANCE_INFO_LEN];
    _mav_put_uint32_t(buf, 0, dance_size);
    _mav_put_uint8_t(buf, 4, cmd);
    _mav_put_char_array(buf, 5, dance_name, 64);
    _mav_put_uint8_t_array(buf, 69, dance_md5, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DANCE_INFO_LEN);
#else
    mavlink_dance_info_t packet;
    packet.dance_size = dance_size;
    packet.cmd = cmd;
    mav_array_memcpy(packet.dance_name, dance_name, sizeof(char)*64);
    mav_array_memcpy(packet.dance_md5, dance_md5, sizeof(uint8_t)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DANCE_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DANCE_INFO;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DANCE_INFO_MIN_LEN, MAVLINK_MSG_ID_DANCE_INFO_LEN, MAVLINK_MSG_ID_DANCE_INFO_CRC);
}

/**
 * @brief Pack a dance_info message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param cmd  cmd
 * @param dance_name  dance name
 * @param dance_size  dance size
 * @param dance_md5  dance md5
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_dance_info_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t cmd,const char *dance_name,uint32_t dance_size,const uint8_t *dance_md5)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DANCE_INFO_LEN];
    _mav_put_uint32_t(buf, 0, dance_size);
    _mav_put_uint8_t(buf, 4, cmd);
    _mav_put_char_array(buf, 5, dance_name, 64);
    _mav_put_uint8_t_array(buf, 69, dance_md5, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DANCE_INFO_LEN);
#else
    mavlink_dance_info_t packet;
    packet.dance_size = dance_size;
    packet.cmd = cmd;
    mav_array_memcpy(packet.dance_name, dance_name, sizeof(char)*64);
    mav_array_memcpy(packet.dance_md5, dance_md5, sizeof(uint8_t)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DANCE_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DANCE_INFO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DANCE_INFO_MIN_LEN, MAVLINK_MSG_ID_DANCE_INFO_LEN, MAVLINK_MSG_ID_DANCE_INFO_CRC);
}

/**
 * @brief Encode a dance_info struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param dance_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_dance_info_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_dance_info_t* dance_info)
{
    return mavlink_msg_dance_info_pack(system_id, component_id, msg, dance_info->cmd, dance_info->dance_name, dance_info->dance_size, dance_info->dance_md5);
}

/**
 * @brief Encode a dance_info struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param dance_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_dance_info_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_dance_info_t* dance_info)
{
    return mavlink_msg_dance_info_pack_chan(system_id, component_id, chan, msg, dance_info->cmd, dance_info->dance_name, dance_info->dance_size, dance_info->dance_md5);
}

/**
 * @brief Send a dance_info message
 * @param chan MAVLink channel to send the message
 *
 * @param cmd  cmd
 * @param dance_name  dance name
 * @param dance_size  dance size
 * @param dance_md5  dance md5
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_dance_info_send(mavlink_channel_t chan, uint8_t cmd, const char *dance_name, uint32_t dance_size, const uint8_t *dance_md5)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DANCE_INFO_LEN];
    _mav_put_uint32_t(buf, 0, dance_size);
    _mav_put_uint8_t(buf, 4, cmd);
    _mav_put_char_array(buf, 5, dance_name, 64);
    _mav_put_uint8_t_array(buf, 69, dance_md5, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DANCE_INFO, buf, MAVLINK_MSG_ID_DANCE_INFO_MIN_LEN, MAVLINK_MSG_ID_DANCE_INFO_LEN, MAVLINK_MSG_ID_DANCE_INFO_CRC);
#else
    mavlink_dance_info_t packet;
    packet.dance_size = dance_size;
    packet.cmd = cmd;
    mav_array_memcpy(packet.dance_name, dance_name, sizeof(char)*64);
    mav_array_memcpy(packet.dance_md5, dance_md5, sizeof(uint8_t)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DANCE_INFO, (const char *)&packet, MAVLINK_MSG_ID_DANCE_INFO_MIN_LEN, MAVLINK_MSG_ID_DANCE_INFO_LEN, MAVLINK_MSG_ID_DANCE_INFO_CRC);
#endif
}

/**
 * @brief Send a dance_info message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_dance_info_send_struct(mavlink_channel_t chan, const mavlink_dance_info_t* dance_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_dance_info_send(chan, dance_info->cmd, dance_info->dance_name, dance_info->dance_size, dance_info->dance_md5);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DANCE_INFO, (const char *)dance_info, MAVLINK_MSG_ID_DANCE_INFO_MIN_LEN, MAVLINK_MSG_ID_DANCE_INFO_LEN, MAVLINK_MSG_ID_DANCE_INFO_CRC);
#endif
}

#if MAVLINK_MSG_ID_DANCE_INFO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_dance_info_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t cmd, const char *dance_name, uint32_t dance_size, const uint8_t *dance_md5)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, dance_size);
    _mav_put_uint8_t(buf, 4, cmd);
    _mav_put_char_array(buf, 5, dance_name, 64);
    _mav_put_uint8_t_array(buf, 69, dance_md5, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DANCE_INFO, buf, MAVLINK_MSG_ID_DANCE_INFO_MIN_LEN, MAVLINK_MSG_ID_DANCE_INFO_LEN, MAVLINK_MSG_ID_DANCE_INFO_CRC);
#else
    mavlink_dance_info_t *packet = (mavlink_dance_info_t *)msgbuf;
    packet->dance_size = dance_size;
    packet->cmd = cmd;
    mav_array_memcpy(packet->dance_name, dance_name, sizeof(char)*64);
    mav_array_memcpy(packet->dance_md5, dance_md5, sizeof(uint8_t)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DANCE_INFO, (const char *)packet, MAVLINK_MSG_ID_DANCE_INFO_MIN_LEN, MAVLINK_MSG_ID_DANCE_INFO_LEN, MAVLINK_MSG_ID_DANCE_INFO_CRC);
#endif
}
#endif

#endif

// MESSAGE DANCE_INFO UNPACKING


/**
 * @brief Get field cmd from dance_info message
 *
 * @return  cmd
 */
static inline uint8_t mavlink_msg_dance_info_get_cmd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field dance_name from dance_info message
 *
 * @return  dance name
 */
static inline uint16_t mavlink_msg_dance_info_get_dance_name(const mavlink_message_t* msg, char *dance_name)
{
    return _MAV_RETURN_char_array(msg, dance_name, 64,  5);
}

/**
 * @brief Get field dance_size from dance_info message
 *
 * @return  dance size
 */
static inline uint32_t mavlink_msg_dance_info_get_dance_size(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field dance_md5 from dance_info message
 *
 * @return  dance md5
 */
static inline uint16_t mavlink_msg_dance_info_get_dance_md5(const mavlink_message_t* msg, uint8_t *dance_md5)
{
    return _MAV_RETURN_uint8_t_array(msg, dance_md5, 16,  69);
}

/**
 * @brief Decode a dance_info message into a struct
 *
 * @param msg The message to decode
 * @param dance_info C-struct to decode the message contents into
 */
static inline void mavlink_msg_dance_info_decode(const mavlink_message_t* msg, mavlink_dance_info_t* dance_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    dance_info->dance_size = mavlink_msg_dance_info_get_dance_size(msg);
    dance_info->cmd = mavlink_msg_dance_info_get_cmd(msg);
    mavlink_msg_dance_info_get_dance_name(msg, dance_info->dance_name);
    mavlink_msg_dance_info_get_dance_md5(msg, dance_info->dance_md5);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DANCE_INFO_LEN? msg->len : MAVLINK_MSG_ID_DANCE_INFO_LEN;
        memset(dance_info, 0, MAVLINK_MSG_ID_DANCE_INFO_LEN);
    memcpy(dance_info, _MAV_PAYLOAD(msg), len);
#endif
}
