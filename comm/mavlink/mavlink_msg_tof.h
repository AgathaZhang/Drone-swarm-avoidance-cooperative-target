#pragma once
// MESSAGE TOF PACKING

#define MAVLINK_MSG_ID_TOF 164


typedef struct __mavlink_tof_t {
 uint64_t timestamp; /*<  tof timestamp*/
 int32_t height; /*<  height unit:mm*/
 uint8_t quality; /*<  0 invalid,1 valid*/
} mavlink_tof_t;

#define MAVLINK_MSG_ID_TOF_LEN 13
#define MAVLINK_MSG_ID_TOF_MIN_LEN 13
#define MAVLINK_MSG_ID_164_LEN 13
#define MAVLINK_MSG_ID_164_MIN_LEN 13

#define MAVLINK_MSG_ID_TOF_CRC 194
#define MAVLINK_MSG_ID_164_CRC 194



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TOF { \
    164, \
    "TOF", \
    3, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_tof_t, timestamp) }, \
         { "quality", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_tof_t, quality) }, \
         { "height", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_tof_t, height) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TOF { \
    "TOF", \
    3, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_tof_t, timestamp) }, \
         { "quality", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_tof_t, quality) }, \
         { "height", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_tof_t, height) }, \
         } \
}
#endif

/**
 * @brief Pack a tof message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp  tof timestamp
 * @param quality  0 invalid,1 valid
 * @param height  height unit:mm
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tof_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint8_t quality, int32_t height)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TOF_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_int32_t(buf, 8, height);
    _mav_put_uint8_t(buf, 12, quality);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TOF_LEN);
#else
    mavlink_tof_t packet;
    packet.timestamp = timestamp;
    packet.height = height;
    packet.quality = quality;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TOF_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TOF;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TOF_MIN_LEN, MAVLINK_MSG_ID_TOF_LEN, MAVLINK_MSG_ID_TOF_CRC);
}

/**
 * @brief Pack a tof message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp  tof timestamp
 * @param quality  0 invalid,1 valid
 * @param height  height unit:mm
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tof_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint8_t quality,int32_t height)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TOF_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_int32_t(buf, 8, height);
    _mav_put_uint8_t(buf, 12, quality);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TOF_LEN);
#else
    mavlink_tof_t packet;
    packet.timestamp = timestamp;
    packet.height = height;
    packet.quality = quality;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TOF_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TOF;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TOF_MIN_LEN, MAVLINK_MSG_ID_TOF_LEN, MAVLINK_MSG_ID_TOF_CRC);
}

/**
 * @brief Encode a tof struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param tof C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tof_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_tof_t* tof)
{
    return mavlink_msg_tof_pack(system_id, component_id, msg, tof->timestamp, tof->quality, tof->height);
}

/**
 * @brief Encode a tof struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param tof C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tof_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_tof_t* tof)
{
    return mavlink_msg_tof_pack_chan(system_id, component_id, chan, msg, tof->timestamp, tof->quality, tof->height);
}

/**
 * @brief Send a tof message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp  tof timestamp
 * @param quality  0 invalid,1 valid
 * @param height  height unit:mm
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_tof_send(mavlink_channel_t chan, uint64_t timestamp, uint8_t quality, int32_t height)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TOF_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_int32_t(buf, 8, height);
    _mav_put_uint8_t(buf, 12, quality);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TOF, buf, MAVLINK_MSG_ID_TOF_MIN_LEN, MAVLINK_MSG_ID_TOF_LEN, MAVLINK_MSG_ID_TOF_CRC);
#else
    mavlink_tof_t packet;
    packet.timestamp = timestamp;
    packet.height = height;
    packet.quality = quality;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TOF, (const char *)&packet, MAVLINK_MSG_ID_TOF_MIN_LEN, MAVLINK_MSG_ID_TOF_LEN, MAVLINK_MSG_ID_TOF_CRC);
#endif
}

/**
 * @brief Send a tof message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_tof_send_struct(mavlink_channel_t chan, const mavlink_tof_t* tof)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_tof_send(chan, tof->timestamp, tof->quality, tof->height);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TOF, (const char *)tof, MAVLINK_MSG_ID_TOF_MIN_LEN, MAVLINK_MSG_ID_TOF_LEN, MAVLINK_MSG_ID_TOF_CRC);
#endif
}

#if MAVLINK_MSG_ID_TOF_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_tof_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint8_t quality, int32_t height)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_int32_t(buf, 8, height);
    _mav_put_uint8_t(buf, 12, quality);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TOF, buf, MAVLINK_MSG_ID_TOF_MIN_LEN, MAVLINK_MSG_ID_TOF_LEN, MAVLINK_MSG_ID_TOF_CRC);
#else
    mavlink_tof_t *packet = (mavlink_tof_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->height = height;
    packet->quality = quality;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TOF, (const char *)packet, MAVLINK_MSG_ID_TOF_MIN_LEN, MAVLINK_MSG_ID_TOF_LEN, MAVLINK_MSG_ID_TOF_CRC);
#endif
}
#endif

#endif

// MESSAGE TOF UNPACKING


/**
 * @brief Get field timestamp from tof message
 *
 * @return  tof timestamp
 */
static inline uint64_t mavlink_msg_tof_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field quality from tof message
 *
 * @return  0 invalid,1 valid
 */
static inline uint8_t mavlink_msg_tof_get_quality(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field height from tof message
 *
 * @return  height unit:mm
 */
static inline int32_t mavlink_msg_tof_get_height(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Decode a tof message into a struct
 *
 * @param msg The message to decode
 * @param tof C-struct to decode the message contents into
 */
static inline void mavlink_msg_tof_decode(const mavlink_message_t* msg, mavlink_tof_t* tof)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    tof->timestamp = mavlink_msg_tof_get_timestamp(msg);
    tof->height = mavlink_msg_tof_get_height(msg);
    tof->quality = mavlink_msg_tof_get_quality(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TOF_LEN? msg->len : MAVLINK_MSG_ID_TOF_LEN;
        memset(tof, 0, MAVLINK_MSG_ID_TOF_LEN);
    memcpy(tof, _MAV_PAYLOAD(msg), len);
#endif
}
