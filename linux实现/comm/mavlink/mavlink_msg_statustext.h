#pragma once
// MESSAGE STATUSTEXT PACKING

#define MAVLINK_MSG_ID_STATUSTEXT 253


typedef struct __mavlink_statustext_t {
 uint32_t id; /*<  Text message id.*/
 uint8_t severity; /*<  Severity of status. Relies on the definitions within RFC-5424.*/
 char text[50]; /*<  Status text message, without null termination character*/
} mavlink_statustext_t;

#define MAVLINK_MSG_ID_STATUSTEXT_LEN 55
#define MAVLINK_MSG_ID_STATUSTEXT_MIN_LEN 55
#define MAVLINK_MSG_ID_253_LEN 55
#define MAVLINK_MSG_ID_253_MIN_LEN 55

#define MAVLINK_MSG_ID_STATUSTEXT_CRC 245
#define MAVLINK_MSG_ID_253_CRC 245

#define MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN 50

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_STATUSTEXT { \
    253, \
    "STATUSTEXT", \
    3, \
    {  { "id", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_statustext_t, id) }, \
         { "severity", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_statustext_t, severity) }, \
         { "text", NULL, MAVLINK_TYPE_CHAR, 50, 5, offsetof(mavlink_statustext_t, text) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_STATUSTEXT { \
    "STATUSTEXT", \
    3, \
    {  { "id", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_statustext_t, id) }, \
         { "severity", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_statustext_t, severity) }, \
         { "text", NULL, MAVLINK_TYPE_CHAR, 50, 5, offsetof(mavlink_statustext_t, text) }, \
         } \
}
#endif

/**
 * @brief Pack a statustext message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param id  Text message id.
 * @param severity  Severity of status. Relies on the definitions within RFC-5424.
 * @param text  Status text message, without null termination character
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_statustext_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t id, uint8_t severity, const char *text)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STATUSTEXT_LEN];
    _mav_put_uint32_t(buf, 0, id);
    _mav_put_uint8_t(buf, 4, severity);
    _mav_put_char_array(buf, 5, text, 50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STATUSTEXT_LEN);
#else
    mavlink_statustext_t packet;
    packet.id = id;
    packet.severity = severity;
    mav_array_memcpy(packet.text, text, sizeof(char)*50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STATUSTEXT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STATUSTEXT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_STATUSTEXT_MIN_LEN, MAVLINK_MSG_ID_STATUSTEXT_LEN, MAVLINK_MSG_ID_STATUSTEXT_CRC);
}

/**
 * @brief Pack a statustext message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param id  Text message id.
 * @param severity  Severity of status. Relies on the definitions within RFC-5424.
 * @param text  Status text message, without null termination character
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_statustext_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t id,uint8_t severity,const char *text)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STATUSTEXT_LEN];
    _mav_put_uint32_t(buf, 0, id);
    _mav_put_uint8_t(buf, 4, severity);
    _mav_put_char_array(buf, 5, text, 50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STATUSTEXT_LEN);
#else
    mavlink_statustext_t packet;
    packet.id = id;
    packet.severity = severity;
    mav_array_memcpy(packet.text, text, sizeof(char)*50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STATUSTEXT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STATUSTEXT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_STATUSTEXT_MIN_LEN, MAVLINK_MSG_ID_STATUSTEXT_LEN, MAVLINK_MSG_ID_STATUSTEXT_CRC);
}

/**
 * @brief Encode a statustext struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param statustext C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_statustext_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_statustext_t* statustext)
{
    return mavlink_msg_statustext_pack(system_id, component_id, msg, statustext->id, statustext->severity, statustext->text);
}

/**
 * @brief Encode a statustext struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param statustext C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_statustext_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_statustext_t* statustext)
{
    return mavlink_msg_statustext_pack_chan(system_id, component_id, chan, msg, statustext->id, statustext->severity, statustext->text);
}

/**
 * @brief Send a statustext message
 * @param chan MAVLink channel to send the message
 *
 * @param id  Text message id.
 * @param severity  Severity of status. Relies on the definitions within RFC-5424.
 * @param text  Status text message, without null termination character
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_statustext_send(mavlink_channel_t chan, uint32_t id, uint8_t severity, const char *text)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STATUSTEXT_LEN];
    _mav_put_uint32_t(buf, 0, id);
    _mav_put_uint8_t(buf, 4, severity);
    _mav_put_char_array(buf, 5, text, 50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATUSTEXT, buf, MAVLINK_MSG_ID_STATUSTEXT_MIN_LEN, MAVLINK_MSG_ID_STATUSTEXT_LEN, MAVLINK_MSG_ID_STATUSTEXT_CRC);
#else
    mavlink_statustext_t packet;
    packet.id = id;
    packet.severity = severity;
    mav_array_memcpy(packet.text, text, sizeof(char)*50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATUSTEXT, (const char *)&packet, MAVLINK_MSG_ID_STATUSTEXT_MIN_LEN, MAVLINK_MSG_ID_STATUSTEXT_LEN, MAVLINK_MSG_ID_STATUSTEXT_CRC);
#endif
}

/**
 * @brief Send a statustext message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_statustext_send_struct(mavlink_channel_t chan, const mavlink_statustext_t* statustext)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_statustext_send(chan, statustext->id, statustext->severity, statustext->text);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATUSTEXT, (const char *)statustext, MAVLINK_MSG_ID_STATUSTEXT_MIN_LEN, MAVLINK_MSG_ID_STATUSTEXT_LEN, MAVLINK_MSG_ID_STATUSTEXT_CRC);
#endif
}

#if MAVLINK_MSG_ID_STATUSTEXT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_statustext_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t id, uint8_t severity, const char *text)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, id);
    _mav_put_uint8_t(buf, 4, severity);
    _mav_put_char_array(buf, 5, text, 50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATUSTEXT, buf, MAVLINK_MSG_ID_STATUSTEXT_MIN_LEN, MAVLINK_MSG_ID_STATUSTEXT_LEN, MAVLINK_MSG_ID_STATUSTEXT_CRC);
#else
    mavlink_statustext_t *packet = (mavlink_statustext_t *)msgbuf;
    packet->id = id;
    packet->severity = severity;
    mav_array_memcpy(packet->text, text, sizeof(char)*50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATUSTEXT, (const char *)packet, MAVLINK_MSG_ID_STATUSTEXT_MIN_LEN, MAVLINK_MSG_ID_STATUSTEXT_LEN, MAVLINK_MSG_ID_STATUSTEXT_CRC);
#endif
}
#endif

#endif

// MESSAGE STATUSTEXT UNPACKING


/**
 * @brief Get field id from statustext message
 *
 * @return  Text message id.
 */
static inline uint32_t mavlink_msg_statustext_get_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field severity from statustext message
 *
 * @return  Severity of status. Relies on the definitions within RFC-5424.
 */
static inline uint8_t mavlink_msg_statustext_get_severity(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field text from statustext message
 *
 * @return  Status text message, without null termination character
 */
static inline uint16_t mavlink_msg_statustext_get_text(const mavlink_message_t* msg, char *text)
{
    return _MAV_RETURN_char_array(msg, text, 50,  5);
}

/**
 * @brief Decode a statustext message into a struct
 *
 * @param msg The message to decode
 * @param statustext C-struct to decode the message contents into
 */
static inline void mavlink_msg_statustext_decode(const mavlink_message_t* msg, mavlink_statustext_t* statustext)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    statustext->id = mavlink_msg_statustext_get_id(msg);
    statustext->severity = mavlink_msg_statustext_get_severity(msg);
    mavlink_msg_statustext_get_text(msg, statustext->text);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_STATUSTEXT_LEN? msg->len : MAVLINK_MSG_ID_STATUSTEXT_LEN;
        memset(statustext, 0, MAVLINK_MSG_ID_STATUSTEXT_LEN);
    memcpy(statustext, _MAV_PAYLOAD(msg), len);
#endif
}
