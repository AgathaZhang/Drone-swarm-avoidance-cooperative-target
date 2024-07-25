#pragma once
// MESSAGE MOUNT_STATUS PACKING

#define MAVLINK_MSG_ID_MOUNT_STATUS 158


typedef struct __mavlink_mount_status_t {
 int16_t roll; /*< [cdeg] Pitch.*/
 int16_t pitch; /*< [cdeg] Roll.*/
 int16_t yaw; /*< [cdeg] Yaw.*/
 int16_t error_yaw; /*< [cdeg] error_yaw.*/
 int16_t zoom_rate; /*< [0.1] zoom_rate.*/
 uint8_t target_system; /*<  System ID.*/
 uint8_t target_component; /*<  Component ID.*/
} mavlink_mount_status_t;

#define MAVLINK_MSG_ID_MOUNT_STATUS_LEN 12
#define MAVLINK_MSG_ID_MOUNT_STATUS_MIN_LEN 12
#define MAVLINK_MSG_ID_158_LEN 12
#define MAVLINK_MSG_ID_158_MIN_LEN 12

#define MAVLINK_MSG_ID_MOUNT_STATUS_CRC 150
#define MAVLINK_MSG_ID_158_CRC 150



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MOUNT_STATUS { \
    158, \
    "MOUNT_STATUS", \
    7, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_mount_status_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_mount_status_t, target_component) }, \
         { "roll", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_mount_status_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_mount_status_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_mount_status_t, yaw) }, \
         { "error_yaw", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_mount_status_t, error_yaw) }, \
         { "zoom_rate", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_mount_status_t, zoom_rate) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MOUNT_STATUS { \
    "MOUNT_STATUS", \
    7, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_mount_status_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_mount_status_t, target_component) }, \
         { "roll", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_mount_status_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_mount_status_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_mount_status_t, yaw) }, \
         { "error_yaw", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_mount_status_t, error_yaw) }, \
         { "zoom_rate", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_mount_status_t, zoom_rate) }, \
         } \
}
#endif

/**
 * @brief Pack a mount_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param roll [cdeg] Pitch.
 * @param pitch [cdeg] Roll.
 * @param yaw [cdeg] Yaw.
 * @param error_yaw [cdeg] error_yaw.
 * @param zoom_rate [0.1] zoom_rate.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mount_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, int16_t roll, int16_t pitch, int16_t yaw, int16_t error_yaw, int16_t zoom_rate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOUNT_STATUS_LEN];
    _mav_put_int16_t(buf, 0, roll);
    _mav_put_int16_t(buf, 2, pitch);
    _mav_put_int16_t(buf, 4, yaw);
    _mav_put_int16_t(buf, 6, error_yaw);
    _mav_put_int16_t(buf, 8, zoom_rate);
    _mav_put_uint8_t(buf, 10, target_system);
    _mav_put_uint8_t(buf, 11, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOUNT_STATUS_LEN);
#else
    mavlink_mount_status_t packet;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.error_yaw = error_yaw;
    packet.zoom_rate = zoom_rate;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOUNT_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOUNT_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MOUNT_STATUS_MIN_LEN, MAVLINK_MSG_ID_MOUNT_STATUS_LEN, MAVLINK_MSG_ID_MOUNT_STATUS_CRC);
}

/**
 * @brief Pack a mount_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param roll [cdeg] Pitch.
 * @param pitch [cdeg] Roll.
 * @param yaw [cdeg] Yaw.
 * @param error_yaw [cdeg] error_yaw.
 * @param zoom_rate [0.1] zoom_rate.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mount_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,int16_t roll,int16_t pitch,int16_t yaw,int16_t error_yaw,int16_t zoom_rate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOUNT_STATUS_LEN];
    _mav_put_int16_t(buf, 0, roll);
    _mav_put_int16_t(buf, 2, pitch);
    _mav_put_int16_t(buf, 4, yaw);
    _mav_put_int16_t(buf, 6, error_yaw);
    _mav_put_int16_t(buf, 8, zoom_rate);
    _mav_put_uint8_t(buf, 10, target_system);
    _mav_put_uint8_t(buf, 11, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOUNT_STATUS_LEN);
#else
    mavlink_mount_status_t packet;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.error_yaw = error_yaw;
    packet.zoom_rate = zoom_rate;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOUNT_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOUNT_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MOUNT_STATUS_MIN_LEN, MAVLINK_MSG_ID_MOUNT_STATUS_LEN, MAVLINK_MSG_ID_MOUNT_STATUS_CRC);
}

/**
 * @brief Encode a mount_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mount_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mount_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mount_status_t* mount_status)
{
    return mavlink_msg_mount_status_pack(system_id, component_id, msg, mount_status->target_system, mount_status->target_component, mount_status->roll, mount_status->pitch, mount_status->yaw, mount_status->error_yaw, mount_status->zoom_rate);
}

/**
 * @brief Encode a mount_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mount_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mount_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mount_status_t* mount_status)
{
    return mavlink_msg_mount_status_pack_chan(system_id, component_id, chan, msg, mount_status->target_system, mount_status->target_component, mount_status->roll, mount_status->pitch, mount_status->yaw, mount_status->error_yaw, mount_status->zoom_rate);
}

/**
 * @brief Send a mount_status message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param roll [cdeg] Pitch.
 * @param pitch [cdeg] Roll.
 * @param yaw [cdeg] Yaw.
 * @param error_yaw [cdeg] error_yaw.
 * @param zoom_rate [0.1] zoom_rate.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mount_status_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, int16_t roll, int16_t pitch, int16_t yaw, int16_t error_yaw, int16_t zoom_rate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOUNT_STATUS_LEN];
    _mav_put_int16_t(buf, 0, roll);
    _mav_put_int16_t(buf, 2, pitch);
    _mav_put_int16_t(buf, 4, yaw);
    _mav_put_int16_t(buf, 6, error_yaw);
    _mav_put_int16_t(buf, 8, zoom_rate);
    _mav_put_uint8_t(buf, 10, target_system);
    _mav_put_uint8_t(buf, 11, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOUNT_STATUS, buf, MAVLINK_MSG_ID_MOUNT_STATUS_MIN_LEN, MAVLINK_MSG_ID_MOUNT_STATUS_LEN, MAVLINK_MSG_ID_MOUNT_STATUS_CRC);
#else
    mavlink_mount_status_t packet;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.error_yaw = error_yaw;
    packet.zoom_rate = zoom_rate;
    packet.target_system = target_system;
    packet.target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOUNT_STATUS, (const char *)&packet, MAVLINK_MSG_ID_MOUNT_STATUS_MIN_LEN, MAVLINK_MSG_ID_MOUNT_STATUS_LEN, MAVLINK_MSG_ID_MOUNT_STATUS_CRC);
#endif
}

/**
 * @brief Send a mount_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mount_status_send_struct(mavlink_channel_t chan, const mavlink_mount_status_t* mount_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mount_status_send(chan, mount_status->target_system, mount_status->target_component, mount_status->roll, mount_status->pitch, mount_status->yaw, mount_status->error_yaw, mount_status->zoom_rate);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOUNT_STATUS, (const char *)mount_status, MAVLINK_MSG_ID_MOUNT_STATUS_MIN_LEN, MAVLINK_MSG_ID_MOUNT_STATUS_LEN, MAVLINK_MSG_ID_MOUNT_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_MOUNT_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mount_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, int16_t roll, int16_t pitch, int16_t yaw, int16_t error_yaw, int16_t zoom_rate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int16_t(buf, 0, roll);
    _mav_put_int16_t(buf, 2, pitch);
    _mav_put_int16_t(buf, 4, yaw);
    _mav_put_int16_t(buf, 6, error_yaw);
    _mav_put_int16_t(buf, 8, zoom_rate);
    _mav_put_uint8_t(buf, 10, target_system);
    _mav_put_uint8_t(buf, 11, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOUNT_STATUS, buf, MAVLINK_MSG_ID_MOUNT_STATUS_MIN_LEN, MAVLINK_MSG_ID_MOUNT_STATUS_LEN, MAVLINK_MSG_ID_MOUNT_STATUS_CRC);
#else
    mavlink_mount_status_t *packet = (mavlink_mount_status_t *)msgbuf;
    packet->roll = roll;
    packet->pitch = pitch;
    packet->yaw = yaw;
    packet->error_yaw = error_yaw;
    packet->zoom_rate = zoom_rate;
    packet->target_system = target_system;
    packet->target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOUNT_STATUS, (const char *)packet, MAVLINK_MSG_ID_MOUNT_STATUS_MIN_LEN, MAVLINK_MSG_ID_MOUNT_STATUS_LEN, MAVLINK_MSG_ID_MOUNT_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE MOUNT_STATUS UNPACKING


/**
 * @brief Get field target_system from mount_status message
 *
 * @return  System ID.
 */
static inline uint8_t mavlink_msg_mount_status_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field target_component from mount_status message
 *
 * @return  Component ID.
 */
static inline uint8_t mavlink_msg_mount_status_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Get field roll from mount_status message
 *
 * @return [cdeg] Pitch.
 */
static inline int16_t mavlink_msg_mount_status_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field pitch from mount_status message
 *
 * @return [cdeg] Roll.
 */
static inline int16_t mavlink_msg_mount_status_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  2);
}

/**
 * @brief Get field yaw from mount_status message
 *
 * @return [cdeg] Yaw.
 */
static inline int16_t mavlink_msg_mount_status_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  4);
}

/**
 * @brief Get field error_yaw from mount_status message
 *
 * @return [cdeg] error_yaw.
 */
static inline int16_t mavlink_msg_mount_status_get_error_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  6);
}

/**
 * @brief Get field zoom_rate from mount_status message
 *
 * @return [0.1] zoom_rate.
 */
static inline int16_t mavlink_msg_mount_status_get_zoom_rate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Decode a mount_status message into a struct
 *
 * @param msg The message to decode
 * @param mount_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_mount_status_decode(const mavlink_message_t* msg, mavlink_mount_status_t* mount_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mount_status->roll = mavlink_msg_mount_status_get_roll(msg);
    mount_status->pitch = mavlink_msg_mount_status_get_pitch(msg);
    mount_status->yaw = mavlink_msg_mount_status_get_yaw(msg);
    mount_status->error_yaw = mavlink_msg_mount_status_get_error_yaw(msg);
    mount_status->zoom_rate = mavlink_msg_mount_status_get_zoom_rate(msg);
    mount_status->target_system = mavlink_msg_mount_status_get_target_system(msg);
    mount_status->target_component = mavlink_msg_mount_status_get_target_component(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MOUNT_STATUS_LEN? msg->len : MAVLINK_MSG_ID_MOUNT_STATUS_LEN;
        memset(mount_status, 0, MAVLINK_MSG_ID_MOUNT_STATUS_LEN);
    memcpy(mount_status, _MAV_PAYLOAD(msg), len);
#endif
}
