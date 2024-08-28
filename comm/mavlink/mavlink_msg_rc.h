#pragma once
// MESSAGE RC PACKING

#define MAVLINK_MSG_ID_RC 232


typedef struct __mavlink_rc_t {
 uint16_t values[8]; /*<  RC 8ch */
} mavlink_rc_t;

#define MAVLINK_MSG_ID_RC_LEN 16
#define MAVLINK_MSG_ID_RC_MIN_LEN 16
#define MAVLINK_MSG_ID_232_LEN 16
#define MAVLINK_MSG_ID_232_MIN_LEN 16

#define MAVLINK_MSG_ID_RC_CRC 97
#define MAVLINK_MSG_ID_232_CRC 97

#define MAVLINK_MSG_RC_FIELD_VALUES_LEN 8

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_RC { \
    232, \
    "RC", \
    1, \
    {  { "values", NULL, MAVLINK_TYPE_UINT16_T, 8, 0, offsetof(mavlink_rc_t, values) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_RC { \
    "RC", \
    1, \
    {  { "values", NULL, MAVLINK_TYPE_UINT16_T, 8, 0, offsetof(mavlink_rc_t, values) }, \
         } \
}
#endif

/**
 * @brief Pack a rc message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param values  RC 8ch 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rc_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const uint16_t *values)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RC_LEN];

    _mav_put_uint16_t_array(buf, 0, values, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RC_LEN);
#else
    mavlink_rc_t packet;

    mav_array_memcpy(packet.values, values, sizeof(uint16_t)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RC;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RC_MIN_LEN, MAVLINK_MSG_ID_RC_LEN, MAVLINK_MSG_ID_RC_CRC);
}

/**
 * @brief Pack a rc message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param values  RC 8ch 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rc_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const uint16_t *values)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RC_LEN];

    _mav_put_uint16_t_array(buf, 0, values, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RC_LEN);
#else
    mavlink_rc_t packet;

    mav_array_memcpy(packet.values, values, sizeof(uint16_t)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RC;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RC_MIN_LEN, MAVLINK_MSG_ID_RC_LEN, MAVLINK_MSG_ID_RC_CRC);
}

/**
 * @brief Encode a rc struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rc_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rc_t* rc)
{
    return mavlink_msg_rc_pack(system_id, component_id, msg, rc->values);
}

/**
 * @brief Encode a rc struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rc_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rc_t* rc)
{
    return mavlink_msg_rc_pack_chan(system_id, component_id, chan, msg, rc->values);
}

/**
 * @brief Send a rc message
 * @param chan MAVLink channel to send the message
 *
 * @param values  RC 8ch 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rc_send(mavlink_channel_t chan, const uint16_t *values)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RC_LEN];

    _mav_put_uint16_t_array(buf, 0, values, 8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC, buf, MAVLINK_MSG_ID_RC_MIN_LEN, MAVLINK_MSG_ID_RC_LEN, MAVLINK_MSG_ID_RC_CRC);
#else
    mavlink_rc_t packet;

    mav_array_memcpy(packet.values, values, sizeof(uint16_t)*8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC, (const char *)&packet, MAVLINK_MSG_ID_RC_MIN_LEN, MAVLINK_MSG_ID_RC_LEN, MAVLINK_MSG_ID_RC_CRC);
#endif
}

/**
 * @brief Send a rc message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_rc_send_struct(mavlink_channel_t chan, const mavlink_rc_t* rc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_rc_send(chan, rc->values);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC, (const char *)rc, MAVLINK_MSG_ID_RC_MIN_LEN, MAVLINK_MSG_ID_RC_LEN, MAVLINK_MSG_ID_RC_CRC);
#endif
}

#if MAVLINK_MSG_ID_RC_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_rc_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const uint16_t *values)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _mav_put_uint16_t_array(buf, 0, values, 8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC, buf, MAVLINK_MSG_ID_RC_MIN_LEN, MAVLINK_MSG_ID_RC_LEN, MAVLINK_MSG_ID_RC_CRC);
#else
    mavlink_rc_t *packet = (mavlink_rc_t *)msgbuf;

    mav_array_memcpy(packet->values, values, sizeof(uint16_t)*8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC, (const char *)packet, MAVLINK_MSG_ID_RC_MIN_LEN, MAVLINK_MSG_ID_RC_LEN, MAVLINK_MSG_ID_RC_CRC);
#endif
}
#endif

#endif

// MESSAGE RC UNPACKING


/**
 * @brief Get field values from rc message
 *
 * @return  RC 8ch 
 */
static inline uint16_t mavlink_msg_rc_get_values(const mavlink_message_t* msg, uint16_t *values)
{
    return _MAV_RETURN_uint16_t_array(msg, values, 8,  0);
}

/**
 * @brief Decode a rc message into a struct
 *
 * @param msg The message to decode
 * @param rc C-struct to decode the message contents into
 */
static inline void mavlink_msg_rc_decode(const mavlink_message_t* msg, mavlink_rc_t* rc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_rc_get_values(msg, rc->values);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_RC_LEN? msg->len : MAVLINK_MSG_ID_RC_LEN;
        memset(rc, 0, MAVLINK_MSG_ID_RC_LEN);
    memcpy(rc, _MAV_PAYLOAD(msg), len);
#endif
}
