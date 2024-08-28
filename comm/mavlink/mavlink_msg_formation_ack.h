#pragma once
// MESSAGE FORMATION_ACK PACKING

#define MAVLINK_MSG_ID_FORMATION_ACK 209


typedef struct __mavlink_formation_ack_t {
 uint32_t token; /*<  Command token*/
 uint16_t id; /*<  Drone ID*/
 uint8_t cmd; /*<  Command 1-Takeoff 2-Land 3-Prepare 4-Arm 5-Disarm 6-TimeSync 7-CalMag*/
 uint8_t result; /*<  Result*/
 uint8_t type; /*<  Drone is formation or aux 0-formation 1-aux*/
} mavlink_formation_ack_t;

#define MAVLINK_MSG_ID_FORMATION_ACK_LEN 9
#define MAVLINK_MSG_ID_FORMATION_ACK_MIN_LEN 9
#define MAVLINK_MSG_ID_209_LEN 9
#define MAVLINK_MSG_ID_209_MIN_LEN 9

#define MAVLINK_MSG_ID_FORMATION_ACK_CRC 195
#define MAVLINK_MSG_ID_209_CRC 195



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_FORMATION_ACK { \
    209, \
    "FORMATION_ACK", \
    5, \
    {  { "token", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_formation_ack_t, token) }, \
         { "id", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_formation_ack_t, id) }, \
         { "cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_formation_ack_t, cmd) }, \
         { "result", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_formation_ack_t, result) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_formation_ack_t, type) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_FORMATION_ACK { \
    "FORMATION_ACK", \
    5, \
    {  { "token", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_formation_ack_t, token) }, \
         { "id", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_formation_ack_t, id) }, \
         { "cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_formation_ack_t, cmd) }, \
         { "result", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_formation_ack_t, result) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_formation_ack_t, type) }, \
         } \
}
#endif

/**
 * @brief Pack a formation_ack message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param token  Command token
 * @param id  Drone ID
 * @param cmd  Command 1-Takeoff 2-Land 3-Prepare 4-Arm 5-Disarm 6-TimeSync 7-CalMag
 * @param result  Result
 * @param type  Drone is formation or aux 0-formation 1-aux
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_formation_ack_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t token, uint16_t id, uint8_t cmd, uint8_t result, uint8_t type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FORMATION_ACK_LEN];
    _mav_put_uint32_t(buf, 0, token);
    _mav_put_uint16_t(buf, 4, id);
    _mav_put_uint8_t(buf, 6, cmd);
    _mav_put_uint8_t(buf, 7, result);
    _mav_put_uint8_t(buf, 8, type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FORMATION_ACK_LEN);
#else
    mavlink_formation_ack_t packet;
    packet.token = token;
    packet.id = id;
    packet.cmd = cmd;
    packet.result = result;
    packet.type = type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FORMATION_ACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FORMATION_ACK;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FORMATION_ACK_MIN_LEN, MAVLINK_MSG_ID_FORMATION_ACK_LEN, MAVLINK_MSG_ID_FORMATION_ACK_CRC);
}

/**
 * @brief Pack a formation_ack message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param token  Command token
 * @param id  Drone ID
 * @param cmd  Command 1-Takeoff 2-Land 3-Prepare 4-Arm 5-Disarm 6-TimeSync 7-CalMag
 * @param result  Result
 * @param type  Drone is formation or aux 0-formation 1-aux
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_formation_ack_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t token,uint16_t id,uint8_t cmd,uint8_t result,uint8_t type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FORMATION_ACK_LEN];
    _mav_put_uint32_t(buf, 0, token);
    _mav_put_uint16_t(buf, 4, id);
    _mav_put_uint8_t(buf, 6, cmd);
    _mav_put_uint8_t(buf, 7, result);
    _mav_put_uint8_t(buf, 8, type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FORMATION_ACK_LEN);
#else
    mavlink_formation_ack_t packet;
    packet.token = token;
    packet.id = id;
    packet.cmd = cmd;
    packet.result = result;
    packet.type = type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FORMATION_ACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FORMATION_ACK;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FORMATION_ACK_MIN_LEN, MAVLINK_MSG_ID_FORMATION_ACK_LEN, MAVLINK_MSG_ID_FORMATION_ACK_CRC);
}

/**
 * @brief Encode a formation_ack struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param formation_ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_formation_ack_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_formation_ack_t* formation_ack)
{
    return mavlink_msg_formation_ack_pack(system_id, component_id, msg, formation_ack->token, formation_ack->id, formation_ack->cmd, formation_ack->result, formation_ack->type);
}

/**
 * @brief Encode a formation_ack struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param formation_ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_formation_ack_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_formation_ack_t* formation_ack)
{
    return mavlink_msg_formation_ack_pack_chan(system_id, component_id, chan, msg, formation_ack->token, formation_ack->id, formation_ack->cmd, formation_ack->result, formation_ack->type);
}

/**
 * @brief Send a formation_ack message
 * @param chan MAVLink channel to send the message
 *
 * @param token  Command token
 * @param id  Drone ID
 * @param cmd  Command 1-Takeoff 2-Land 3-Prepare 4-Arm 5-Disarm 6-TimeSync 7-CalMag
 * @param result  Result
 * @param type  Drone is formation or aux 0-formation 1-aux
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_formation_ack_send(mavlink_channel_t chan, uint32_t token, uint16_t id, uint8_t cmd, uint8_t result, uint8_t type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FORMATION_ACK_LEN];
    _mav_put_uint32_t(buf, 0, token);
    _mav_put_uint16_t(buf, 4, id);
    _mav_put_uint8_t(buf, 6, cmd);
    _mav_put_uint8_t(buf, 7, result);
    _mav_put_uint8_t(buf, 8, type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FORMATION_ACK, buf, MAVLINK_MSG_ID_FORMATION_ACK_MIN_LEN, MAVLINK_MSG_ID_FORMATION_ACK_LEN, MAVLINK_MSG_ID_FORMATION_ACK_CRC);
#else
    mavlink_formation_ack_t packet;
    packet.token = token;
    packet.id = id;
    packet.cmd = cmd;
    packet.result = result;
    packet.type = type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FORMATION_ACK, (const char *)&packet, MAVLINK_MSG_ID_FORMATION_ACK_MIN_LEN, MAVLINK_MSG_ID_FORMATION_ACK_LEN, MAVLINK_MSG_ID_FORMATION_ACK_CRC);
#endif
}

/**
 * @brief Send a formation_ack message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_formation_ack_send_struct(mavlink_channel_t chan, const mavlink_formation_ack_t* formation_ack)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_formation_ack_send(chan, formation_ack->token, formation_ack->id, formation_ack->cmd, formation_ack->result, formation_ack->type);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FORMATION_ACK, (const char *)formation_ack, MAVLINK_MSG_ID_FORMATION_ACK_MIN_LEN, MAVLINK_MSG_ID_FORMATION_ACK_LEN, MAVLINK_MSG_ID_FORMATION_ACK_CRC);
#endif
}

#if MAVLINK_MSG_ID_FORMATION_ACK_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_formation_ack_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t token, uint16_t id, uint8_t cmd, uint8_t result, uint8_t type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, token);
    _mav_put_uint16_t(buf, 4, id);
    _mav_put_uint8_t(buf, 6, cmd);
    _mav_put_uint8_t(buf, 7, result);
    _mav_put_uint8_t(buf, 8, type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FORMATION_ACK, buf, MAVLINK_MSG_ID_FORMATION_ACK_MIN_LEN, MAVLINK_MSG_ID_FORMATION_ACK_LEN, MAVLINK_MSG_ID_FORMATION_ACK_CRC);
#else
    mavlink_formation_ack_t *packet = (mavlink_formation_ack_t *)msgbuf;
    packet->token = token;
    packet->id = id;
    packet->cmd = cmd;
    packet->result = result;
    packet->type = type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FORMATION_ACK, (const char *)packet, MAVLINK_MSG_ID_FORMATION_ACK_MIN_LEN, MAVLINK_MSG_ID_FORMATION_ACK_LEN, MAVLINK_MSG_ID_FORMATION_ACK_CRC);
#endif
}
#endif

#endif

// MESSAGE FORMATION_ACK UNPACKING


/**
 * @brief Get field token from formation_ack message
 *
 * @return  Command token
 */
static inline uint32_t mavlink_msg_formation_ack_get_token(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field id from formation_ack message
 *
 * @return  Drone ID
 */
static inline uint16_t mavlink_msg_formation_ack_get_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field cmd from formation_ack message
 *
 * @return  Command 1-Takeoff 2-Land 3-Prepare 4-Arm 5-Disarm 6-TimeSync 7-CalMag
 */
static inline uint8_t mavlink_msg_formation_ack_get_cmd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field result from formation_ack message
 *
 * @return  Result
 */
static inline uint8_t mavlink_msg_formation_ack_get_result(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field type from formation_ack message
 *
 * @return  Drone is formation or aux 0-formation 1-aux
 */
static inline uint8_t mavlink_msg_formation_ack_get_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Decode a formation_ack message into a struct
 *
 * @param msg The message to decode
 * @param formation_ack C-struct to decode the message contents into
 */
static inline void mavlink_msg_formation_ack_decode(const mavlink_message_t* msg, mavlink_formation_ack_t* formation_ack)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    formation_ack->token = mavlink_msg_formation_ack_get_token(msg);
    formation_ack->id = mavlink_msg_formation_ack_get_id(msg);
    formation_ack->cmd = mavlink_msg_formation_ack_get_cmd(msg);
    formation_ack->result = mavlink_msg_formation_ack_get_result(msg);
    formation_ack->type = mavlink_msg_formation_ack_get_type(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_FORMATION_ACK_LEN? msg->len : MAVLINK_MSG_ID_FORMATION_ACK_LEN;
        memset(formation_ack, 0, MAVLINK_MSG_ID_FORMATION_ACK_LEN);
    memcpy(formation_ack, _MAV_PAYLOAD(msg), len);
#endif
}
