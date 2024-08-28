#pragma once
// MESSAGE HEARTBEAT PACKING

#define MAVLINK_MSG_ID_HEARTBEAT 0


typedef struct __mavlink_heartbeat_t {
 uint32_t function_switch; /*<  A bitfield for use for function switch flags.*/
 uint32_t flying_info; /*<  A bitfield for use for flying info flags.*/
 uint16_t serial_num; /*<  Drone serial number.*/
 uint8_t type; /*<  Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)*/
 uint8_t autopilot; /*<  Autopilot type / class. defined in MAV_AUTOPILOT ENUM.*/
 uint8_t flight_mode; /*<  Flight mode, see MAV_FLIGHT_MODE ENUM*/
 uint8_t system_status; /*<  System status, see MAV_STATE ENUM*/
 uint8_t mavlink_version; /*<  MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version*/
} mavlink_heartbeat_t;

#define MAVLINK_MSG_ID_HEARTBEAT_LEN 15
#define MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN 15
#define MAVLINK_MSG_ID_0_LEN 15
#define MAVLINK_MSG_ID_0_MIN_LEN 15

#define MAVLINK_MSG_ID_HEARTBEAT_CRC 200
#define MAVLINK_MSG_ID_0_CRC 200



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_HEARTBEAT { \
    0, \
    "HEARTBEAT", \
    8, \
    {  { "function_switch", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_heartbeat_t, function_switch) }, \
         { "flying_info", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_heartbeat_t, flying_info) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_heartbeat_t, type) }, \
         { "autopilot", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_heartbeat_t, autopilot) }, \
         { "serial_num", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_heartbeat_t, serial_num) }, \
         { "flight_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_heartbeat_t, flight_mode) }, \
         { "system_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_heartbeat_t, system_status) }, \
         { "mavlink_version", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_heartbeat_t, mavlink_version) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_HEARTBEAT { \
    "HEARTBEAT", \
    8, \
    {  { "function_switch", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_heartbeat_t, function_switch) }, \
         { "flying_info", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_heartbeat_t, flying_info) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_heartbeat_t, type) }, \
         { "autopilot", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_heartbeat_t, autopilot) }, \
         { "serial_num", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_heartbeat_t, serial_num) }, \
         { "flight_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_heartbeat_t, flight_mode) }, \
         { "system_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_heartbeat_t, system_status) }, \
         { "mavlink_version", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_heartbeat_t, mavlink_version) }, \
         } \
}
#endif

/**
 * @brief Pack a heartbeat message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param function_switch  A bitfield for use for function switch flags.
 * @param flying_info  A bitfield for use for flying info flags.
 * @param type  Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
 * @param autopilot  Autopilot type / class. defined in MAV_AUTOPILOT ENUM.
 * @param serial_num  Drone serial number.
 * @param flight_mode  Flight mode, see MAV_FLIGHT_MODE ENUM
 * @param system_status  System status, see MAV_STATE ENUM
 * @param mavlink_version  MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_heartbeat_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t function_switch, uint32_t flying_info, uint8_t type, uint8_t autopilot, uint16_t serial_num, uint8_t flight_mode, uint8_t system_status, uint8_t mavlink_version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HEARTBEAT_LEN];
    _mav_put_uint32_t(buf, 0, function_switch);
    _mav_put_uint32_t(buf, 4, flying_info);
    _mav_put_uint16_t(buf, 8, serial_num);
    _mav_put_uint8_t(buf, 10, type);
    _mav_put_uint8_t(buf, 11, autopilot);
    _mav_put_uint8_t(buf, 12, flight_mode);
    _mav_put_uint8_t(buf, 13, system_status);
    _mav_put_uint8_t(buf, 14, mavlink_version);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HEARTBEAT_LEN);
#else
    mavlink_heartbeat_t packet;
    packet.function_switch = function_switch;
    packet.flying_info = flying_info;
    packet.serial_num = serial_num;
    packet.type = type;
    packet.autopilot = autopilot;
    packet.flight_mode = flight_mode;
    packet.system_status = system_status;
    packet.mavlink_version = mavlink_version;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HEARTBEAT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HEARTBEAT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_HEARTBEAT_LEN, MAVLINK_MSG_ID_HEARTBEAT_CRC);
}

/**
 * @brief Pack a heartbeat message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param function_switch  A bitfield for use for function switch flags.
 * @param flying_info  A bitfield for use for flying info flags.
 * @param type  Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
 * @param autopilot  Autopilot type / class. defined in MAV_AUTOPILOT ENUM.
 * @param serial_num  Drone serial number.
 * @param flight_mode  Flight mode, see MAV_FLIGHT_MODE ENUM
 * @param system_status  System status, see MAV_STATE ENUM
 * @param mavlink_version  MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_heartbeat_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t function_switch,uint32_t flying_info,uint8_t type,uint8_t autopilot,uint16_t serial_num,uint8_t flight_mode,uint8_t system_status,uint8_t mavlink_version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HEARTBEAT_LEN];
    _mav_put_uint32_t(buf, 0, function_switch);
    _mav_put_uint32_t(buf, 4, flying_info);
    _mav_put_uint16_t(buf, 8, serial_num);
    _mav_put_uint8_t(buf, 10, type);
    _mav_put_uint8_t(buf, 11, autopilot);
    _mav_put_uint8_t(buf, 12, flight_mode);
    _mav_put_uint8_t(buf, 13, system_status);
    _mav_put_uint8_t(buf, 14, mavlink_version);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HEARTBEAT_LEN);
#else
    mavlink_heartbeat_t packet;
    packet.function_switch = function_switch;
    packet.flying_info = flying_info;
    packet.serial_num = serial_num;
    packet.type = type;
    packet.autopilot = autopilot;
    packet.flight_mode = flight_mode;
    packet.system_status = system_status;
    packet.mavlink_version = mavlink_version;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HEARTBEAT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HEARTBEAT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_HEARTBEAT_LEN, MAVLINK_MSG_ID_HEARTBEAT_CRC);
}

/**
 * @brief Encode a heartbeat struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param heartbeat C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_heartbeat_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_heartbeat_t* heartbeat)
{
    return mavlink_msg_heartbeat_pack(system_id, component_id, msg, heartbeat->function_switch, heartbeat->flying_info, heartbeat->type, heartbeat->autopilot, heartbeat->serial_num, heartbeat->flight_mode, heartbeat->system_status, heartbeat->mavlink_version);
}

/**
 * @brief Encode a heartbeat struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param heartbeat C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_heartbeat_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_heartbeat_t* heartbeat)
{
    return mavlink_msg_heartbeat_pack_chan(system_id, component_id, chan, msg, heartbeat->function_switch, heartbeat->flying_info, heartbeat->type, heartbeat->autopilot, heartbeat->serial_num, heartbeat->flight_mode, heartbeat->system_status, heartbeat->mavlink_version);
}

/**
 * @brief Send a heartbeat message
 * @param chan MAVLink channel to send the message
 *
 * @param function_switch  A bitfield for use for function switch flags.
 * @param flying_info  A bitfield for use for flying info flags.
 * @param type  Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
 * @param autopilot  Autopilot type / class. defined in MAV_AUTOPILOT ENUM.
 * @param serial_num  Drone serial number.
 * @param flight_mode  Flight mode, see MAV_FLIGHT_MODE ENUM
 * @param system_status  System status, see MAV_STATE ENUM
 * @param mavlink_version  MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_heartbeat_send(mavlink_channel_t chan, uint32_t function_switch, uint32_t flying_info, uint8_t type, uint8_t autopilot, uint16_t serial_num, uint8_t flight_mode, uint8_t system_status, uint8_t mavlink_version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HEARTBEAT_LEN];
    _mav_put_uint32_t(buf, 0, function_switch);
    _mav_put_uint32_t(buf, 4, flying_info);
    _mav_put_uint16_t(buf, 8, serial_num);
    _mav_put_uint8_t(buf, 10, type);
    _mav_put_uint8_t(buf, 11, autopilot);
    _mav_put_uint8_t(buf, 12, flight_mode);
    _mav_put_uint8_t(buf, 13, system_status);
    _mav_put_uint8_t(buf, 14, mavlink_version);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HEARTBEAT, buf, MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_HEARTBEAT_LEN, MAVLINK_MSG_ID_HEARTBEAT_CRC);
#else
    mavlink_heartbeat_t packet;
    packet.function_switch = function_switch;
    packet.flying_info = flying_info;
    packet.serial_num = serial_num;
    packet.type = type;
    packet.autopilot = autopilot;
    packet.flight_mode = flight_mode;
    packet.system_status = system_status;
    packet.mavlink_version = mavlink_version;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HEARTBEAT, (const char *)&packet, MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_HEARTBEAT_LEN, MAVLINK_MSG_ID_HEARTBEAT_CRC);
#endif
}

/**
 * @brief Send a heartbeat message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_heartbeat_send_struct(mavlink_channel_t chan, const mavlink_heartbeat_t* heartbeat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_heartbeat_send(chan, heartbeat->function_switch, heartbeat->flying_info, heartbeat->type, heartbeat->autopilot, heartbeat->serial_num, heartbeat->flight_mode, heartbeat->system_status, heartbeat->mavlink_version);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HEARTBEAT, (const char *)heartbeat, MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_HEARTBEAT_LEN, MAVLINK_MSG_ID_HEARTBEAT_CRC);
#endif
}

#if MAVLINK_MSG_ID_HEARTBEAT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_heartbeat_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t function_switch, uint32_t flying_info, uint8_t type, uint8_t autopilot, uint16_t serial_num, uint8_t flight_mode, uint8_t system_status, uint8_t mavlink_version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, function_switch);
    _mav_put_uint32_t(buf, 4, flying_info);
    _mav_put_uint16_t(buf, 8, serial_num);
    _mav_put_uint8_t(buf, 10, type);
    _mav_put_uint8_t(buf, 11, autopilot);
    _mav_put_uint8_t(buf, 12, flight_mode);
    _mav_put_uint8_t(buf, 13, system_status);
    _mav_put_uint8_t(buf, 14, mavlink_version);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HEARTBEAT, buf, MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_HEARTBEAT_LEN, MAVLINK_MSG_ID_HEARTBEAT_CRC);
#else
    mavlink_heartbeat_t *packet = (mavlink_heartbeat_t *)msgbuf;
    packet->function_switch = function_switch;
    packet->flying_info = flying_info;
    packet->serial_num = serial_num;
    packet->type = type;
    packet->autopilot = autopilot;
    packet->flight_mode = flight_mode;
    packet->system_status = system_status;
    packet->mavlink_version = mavlink_version;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HEARTBEAT, (const char *)packet, MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_HEARTBEAT_LEN, MAVLINK_MSG_ID_HEARTBEAT_CRC);
#endif
}
#endif

#endif

// MESSAGE HEARTBEAT UNPACKING


/**
 * @brief Get field function_switch from heartbeat message
 *
 * @return  A bitfield for use for function switch flags.
 */
static inline uint32_t mavlink_msg_heartbeat_get_function_switch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field flying_info from heartbeat message
 *
 * @return  A bitfield for use for flying info flags.
 */
static inline uint32_t mavlink_msg_heartbeat_get_flying_info(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field type from heartbeat message
 *
 * @return  Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
 */
static inline uint8_t mavlink_msg_heartbeat_get_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field autopilot from heartbeat message
 *
 * @return  Autopilot type / class. defined in MAV_AUTOPILOT ENUM.
 */
static inline uint8_t mavlink_msg_heartbeat_get_autopilot(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Get field serial_num from heartbeat message
 *
 * @return  Drone serial number.
 */
static inline uint16_t mavlink_msg_heartbeat_get_serial_num(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field flight_mode from heartbeat message
 *
 * @return  Flight mode, see MAV_FLIGHT_MODE ENUM
 */
static inline uint8_t mavlink_msg_heartbeat_get_flight_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field system_status from heartbeat message
 *
 * @return  System status, see MAV_STATE ENUM
 */
static inline uint8_t mavlink_msg_heartbeat_get_system_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Get field mavlink_version from heartbeat message
 *
 * @return  MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version
 */
static inline uint8_t mavlink_msg_heartbeat_get_mavlink_version(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  14);
}

/**
 * @brief Decode a heartbeat message into a struct
 *
 * @param msg The message to decode
 * @param heartbeat C-struct to decode the message contents into
 */
static inline void mavlink_msg_heartbeat_decode(const mavlink_message_t* msg, mavlink_heartbeat_t* heartbeat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    heartbeat->function_switch = mavlink_msg_heartbeat_get_function_switch(msg);
    heartbeat->flying_info = mavlink_msg_heartbeat_get_flying_info(msg);
    heartbeat->serial_num = mavlink_msg_heartbeat_get_serial_num(msg);
    heartbeat->type = mavlink_msg_heartbeat_get_type(msg);
    heartbeat->autopilot = mavlink_msg_heartbeat_get_autopilot(msg);
    heartbeat->flight_mode = mavlink_msg_heartbeat_get_flight_mode(msg);
    heartbeat->system_status = mavlink_msg_heartbeat_get_system_status(msg);
    heartbeat->mavlink_version = mavlink_msg_heartbeat_get_mavlink_version(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_HEARTBEAT_LEN? msg->len : MAVLINK_MSG_ID_HEARTBEAT_LEN;
        memset(heartbeat, 0, MAVLINK_MSG_ID_HEARTBEAT_LEN);
    memcpy(heartbeat, _MAV_PAYLOAD(msg), len);
#endif
}
