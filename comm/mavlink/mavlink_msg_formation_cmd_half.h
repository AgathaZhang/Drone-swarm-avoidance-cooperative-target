#pragma once
// MESSAGE FORMATION_CMD_HALF PACKING

#define MAVLINK_MSG_ID_FORMATION_CMD_HALF 228


typedef struct __mavlink_formation_cmd_half_t {
 uint64_t utc; /*<  Unix Timestamp in milliseconds*/
 uint32_t token; /*<   Command token*/
 uint32_t no_used[4]; /*<  no used*/
 uint16_t id_start[25]; /*<   Drone ID*/
 uint16_t id_end[25]; /*<   Drone ID*/
 uint16_t data; /*<   Data*/
 uint16_t param; /*<  Param*/
 uint8_t cmd; /*<   see instruction book*/
 uint8_t ack; /*<   Ack 0-NoAck 1-Ack*/
 uint8_t type; /*<   Broadcast or point to point 0-Broadcast 1-Point to Point*/
} mavlink_formation_cmd_half_t;

#define MAVLINK_MSG_ID_FORMATION_CMD_HALF_LEN 135
#define MAVLINK_MSG_ID_FORMATION_CMD_HALF_MIN_LEN 135
#define MAVLINK_MSG_ID_228_LEN 135
#define MAVLINK_MSG_ID_228_MIN_LEN 135

#define MAVLINK_MSG_ID_FORMATION_CMD_HALF_CRC 140
#define MAVLINK_MSG_ID_228_CRC 140

#define MAVLINK_MSG_FORMATION_CMD_HALF_FIELD_NO_USED_LEN 4
#define MAVLINK_MSG_FORMATION_CMD_HALF_FIELD_ID_START_LEN 25
#define MAVLINK_MSG_FORMATION_CMD_HALF_FIELD_ID_END_LEN 25

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_FORMATION_CMD_HALF { \
    228, \
    "FORMATION_CMD_HALF", \
    10, \
    {  { "utc", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_formation_cmd_half_t, utc) }, \
         { "token", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_formation_cmd_half_t, token) }, \
         { "no_used", NULL, MAVLINK_TYPE_UINT32_T, 4, 12, offsetof(mavlink_formation_cmd_half_t, no_used) }, \
         { "id_start", NULL, MAVLINK_TYPE_UINT16_T, 25, 28, offsetof(mavlink_formation_cmd_half_t, id_start) }, \
         { "id_end", NULL, MAVLINK_TYPE_UINT16_T, 25, 78, offsetof(mavlink_formation_cmd_half_t, id_end) }, \
         { "data", NULL, MAVLINK_TYPE_UINT16_T, 0, 128, offsetof(mavlink_formation_cmd_half_t, data) }, \
         { "param", NULL, MAVLINK_TYPE_UINT16_T, 0, 130, offsetof(mavlink_formation_cmd_half_t, param) }, \
         { "cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 132, offsetof(mavlink_formation_cmd_half_t, cmd) }, \
         { "ack", NULL, MAVLINK_TYPE_UINT8_T, 0, 133, offsetof(mavlink_formation_cmd_half_t, ack) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 134, offsetof(mavlink_formation_cmd_half_t, type) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_FORMATION_CMD_HALF { \
    "FORMATION_CMD_HALF", \
    10, \
    {  { "utc", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_formation_cmd_half_t, utc) }, \
         { "token", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_formation_cmd_half_t, token) }, \
         { "no_used", NULL, MAVLINK_TYPE_UINT32_T, 4, 12, offsetof(mavlink_formation_cmd_half_t, no_used) }, \
         { "id_start", NULL, MAVLINK_TYPE_UINT16_T, 25, 28, offsetof(mavlink_formation_cmd_half_t, id_start) }, \
         { "id_end", NULL, MAVLINK_TYPE_UINT16_T, 25, 78, offsetof(mavlink_formation_cmd_half_t, id_end) }, \
         { "data", NULL, MAVLINK_TYPE_UINT16_T, 0, 128, offsetof(mavlink_formation_cmd_half_t, data) }, \
         { "param", NULL, MAVLINK_TYPE_UINT16_T, 0, 130, offsetof(mavlink_formation_cmd_half_t, param) }, \
         { "cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 132, offsetof(mavlink_formation_cmd_half_t, cmd) }, \
         { "ack", NULL, MAVLINK_TYPE_UINT8_T, 0, 133, offsetof(mavlink_formation_cmd_half_t, ack) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 134, offsetof(mavlink_formation_cmd_half_t, type) }, \
         } \
}
#endif

/**
 * @brief Pack a formation_cmd_half message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param utc  Unix Timestamp in milliseconds
 * @param token   Command token
 * @param no_used  no used
 * @param id_start   Drone ID
 * @param id_end   Drone ID
 * @param data   Data
 * @param param  Param
 * @param cmd   see instruction book
 * @param ack   Ack 0-NoAck 1-Ack
 * @param type   Broadcast or point to point 0-Broadcast 1-Point to Point
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_formation_cmd_half_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t utc, uint32_t token, const uint32_t *no_used, const uint16_t *id_start, const uint16_t *id_end, uint16_t data, uint16_t param, uint8_t cmd, uint8_t ack, uint8_t type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FORMATION_CMD_HALF_LEN];
    _mav_put_uint64_t(buf, 0, utc);
    _mav_put_uint32_t(buf, 8, token);
    _mav_put_uint16_t(buf, 128, data);
    _mav_put_uint16_t(buf, 130, param);
    _mav_put_uint8_t(buf, 132, cmd);
    _mav_put_uint8_t(buf, 133, ack);
    _mav_put_uint8_t(buf, 134, type);
    _mav_put_uint32_t_array(buf, 12, no_used, 4);
    _mav_put_uint16_t_array(buf, 28, id_start, 25);
    _mav_put_uint16_t_array(buf, 78, id_end, 25);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FORMATION_CMD_HALF_LEN);
#else
    mavlink_formation_cmd_half_t packet;
    packet.utc = utc;
    packet.token = token;
    packet.data = data;
    packet.param = param;
    packet.cmd = cmd;
    packet.ack = ack;
    packet.type = type;
    mav_array_memcpy(packet.no_used, no_used, sizeof(uint32_t)*4);
    mav_array_memcpy(packet.id_start, id_start, sizeof(uint16_t)*25);
    mav_array_memcpy(packet.id_end, id_end, sizeof(uint16_t)*25);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FORMATION_CMD_HALF_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FORMATION_CMD_HALF;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FORMATION_CMD_HALF_MIN_LEN, MAVLINK_MSG_ID_FORMATION_CMD_HALF_LEN, MAVLINK_MSG_ID_FORMATION_CMD_HALF_CRC);
}

/**
 * @brief Pack a formation_cmd_half message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param utc  Unix Timestamp in milliseconds
 * @param token   Command token
 * @param no_used  no used
 * @param id_start   Drone ID
 * @param id_end   Drone ID
 * @param data   Data
 * @param param  Param
 * @param cmd   see instruction book
 * @param ack   Ack 0-NoAck 1-Ack
 * @param type   Broadcast or point to point 0-Broadcast 1-Point to Point
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_formation_cmd_half_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t utc,uint32_t token,const uint32_t *no_used,const uint16_t *id_start,const uint16_t *id_end,uint16_t data,uint16_t param,uint8_t cmd,uint8_t ack,uint8_t type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FORMATION_CMD_HALF_LEN];
    _mav_put_uint64_t(buf, 0, utc);
    _mav_put_uint32_t(buf, 8, token);
    _mav_put_uint16_t(buf, 128, data);
    _mav_put_uint16_t(buf, 130, param);
    _mav_put_uint8_t(buf, 132, cmd);
    _mav_put_uint8_t(buf, 133, ack);
    _mav_put_uint8_t(buf, 134, type);
    _mav_put_uint32_t_array(buf, 12, no_used, 4);
    _mav_put_uint16_t_array(buf, 28, id_start, 25);
    _mav_put_uint16_t_array(buf, 78, id_end, 25);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FORMATION_CMD_HALF_LEN);
#else
    mavlink_formation_cmd_half_t packet;
    packet.utc = utc;
    packet.token = token;
    packet.data = data;
    packet.param = param;
    packet.cmd = cmd;
    packet.ack = ack;
    packet.type = type;
    mav_array_memcpy(packet.no_used, no_used, sizeof(uint32_t)*4);
    mav_array_memcpy(packet.id_start, id_start, sizeof(uint16_t)*25);
    mav_array_memcpy(packet.id_end, id_end, sizeof(uint16_t)*25);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FORMATION_CMD_HALF_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FORMATION_CMD_HALF;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FORMATION_CMD_HALF_MIN_LEN, MAVLINK_MSG_ID_FORMATION_CMD_HALF_LEN, MAVLINK_MSG_ID_FORMATION_CMD_HALF_CRC);
}

/**
 * @brief Encode a formation_cmd_half struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param formation_cmd_half C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_formation_cmd_half_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_formation_cmd_half_t* formation_cmd_half)
{
    return mavlink_msg_formation_cmd_half_pack(system_id, component_id, msg, formation_cmd_half->utc, formation_cmd_half->token, formation_cmd_half->no_used, formation_cmd_half->id_start, formation_cmd_half->id_end, formation_cmd_half->data, formation_cmd_half->param, formation_cmd_half->cmd, formation_cmd_half->ack, formation_cmd_half->type);
}

/**
 * @brief Encode a formation_cmd_half struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param formation_cmd_half C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_formation_cmd_half_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_formation_cmd_half_t* formation_cmd_half)
{
    return mavlink_msg_formation_cmd_half_pack_chan(system_id, component_id, chan, msg, formation_cmd_half->utc, formation_cmd_half->token, formation_cmd_half->no_used, formation_cmd_half->id_start, formation_cmd_half->id_end, formation_cmd_half->data, formation_cmd_half->param, formation_cmd_half->cmd, formation_cmd_half->ack, formation_cmd_half->type);
}

/**
 * @brief Send a formation_cmd_half message
 * @param chan MAVLink channel to send the message
 *
 * @param utc  Unix Timestamp in milliseconds
 * @param token   Command token
 * @param no_used  no used
 * @param id_start   Drone ID
 * @param id_end   Drone ID
 * @param data   Data
 * @param param  Param
 * @param cmd   see instruction book
 * @param ack   Ack 0-NoAck 1-Ack
 * @param type   Broadcast or point to point 0-Broadcast 1-Point to Point
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_formation_cmd_half_send(mavlink_channel_t chan, uint64_t utc, uint32_t token, const uint32_t *no_used, const uint16_t *id_start, const uint16_t *id_end, uint16_t data, uint16_t param, uint8_t cmd, uint8_t ack, uint8_t type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FORMATION_CMD_HALF_LEN];
    _mav_put_uint64_t(buf, 0, utc);
    _mav_put_uint32_t(buf, 8, token);
    _mav_put_uint16_t(buf, 128, data);
    _mav_put_uint16_t(buf, 130, param);
    _mav_put_uint8_t(buf, 132, cmd);
    _mav_put_uint8_t(buf, 133, ack);
    _mav_put_uint8_t(buf, 134, type);
    _mav_put_uint32_t_array(buf, 12, no_used, 4);
    _mav_put_uint16_t_array(buf, 28, id_start, 25);
    _mav_put_uint16_t_array(buf, 78, id_end, 25);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FORMATION_CMD_HALF, buf, MAVLINK_MSG_ID_FORMATION_CMD_HALF_MIN_LEN, MAVLINK_MSG_ID_FORMATION_CMD_HALF_LEN, MAVLINK_MSG_ID_FORMATION_CMD_HALF_CRC);
#else
    mavlink_formation_cmd_half_t packet;
    packet.utc = utc;
    packet.token = token;
    packet.data = data;
    packet.param = param;
    packet.cmd = cmd;
    packet.ack = ack;
    packet.type = type;
    mav_array_memcpy(packet.no_used, no_used, sizeof(uint32_t)*4);
    mav_array_memcpy(packet.id_start, id_start, sizeof(uint16_t)*25);
    mav_array_memcpy(packet.id_end, id_end, sizeof(uint16_t)*25);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FORMATION_CMD_HALF, (const char *)&packet, MAVLINK_MSG_ID_FORMATION_CMD_HALF_MIN_LEN, MAVLINK_MSG_ID_FORMATION_CMD_HALF_LEN, MAVLINK_MSG_ID_FORMATION_CMD_HALF_CRC);
#endif
}

/**
 * @brief Send a formation_cmd_half message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_formation_cmd_half_send_struct(mavlink_channel_t chan, const mavlink_formation_cmd_half_t* formation_cmd_half)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_formation_cmd_half_send(chan, formation_cmd_half->utc, formation_cmd_half->token, formation_cmd_half->no_used, formation_cmd_half->id_start, formation_cmd_half->id_end, formation_cmd_half->data, formation_cmd_half->param, formation_cmd_half->cmd, formation_cmd_half->ack, formation_cmd_half->type);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FORMATION_CMD_HALF, (const char *)formation_cmd_half, MAVLINK_MSG_ID_FORMATION_CMD_HALF_MIN_LEN, MAVLINK_MSG_ID_FORMATION_CMD_HALF_LEN, MAVLINK_MSG_ID_FORMATION_CMD_HALF_CRC);
#endif
}

#if MAVLINK_MSG_ID_FORMATION_CMD_HALF_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_formation_cmd_half_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t utc, uint32_t token, const uint32_t *no_used, const uint16_t *id_start, const uint16_t *id_end, uint16_t data, uint16_t param, uint8_t cmd, uint8_t ack, uint8_t type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, utc);
    _mav_put_uint32_t(buf, 8, token);
    _mav_put_uint16_t(buf, 128, data);
    _mav_put_uint16_t(buf, 130, param);
    _mav_put_uint8_t(buf, 132, cmd);
    _mav_put_uint8_t(buf, 133, ack);
    _mav_put_uint8_t(buf, 134, type);
    _mav_put_uint32_t_array(buf, 12, no_used, 4);
    _mav_put_uint16_t_array(buf, 28, id_start, 25);
    _mav_put_uint16_t_array(buf, 78, id_end, 25);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FORMATION_CMD_HALF, buf, MAVLINK_MSG_ID_FORMATION_CMD_HALF_MIN_LEN, MAVLINK_MSG_ID_FORMATION_CMD_HALF_LEN, MAVLINK_MSG_ID_FORMATION_CMD_HALF_CRC);
#else
    mavlink_formation_cmd_half_t *packet = (mavlink_formation_cmd_half_t *)msgbuf;
    packet->utc = utc;
    packet->token = token;
    packet->data = data;
    packet->param = param;
    packet->cmd = cmd;
    packet->ack = ack;
    packet->type = type;
    mav_array_memcpy(packet->no_used, no_used, sizeof(uint32_t)*4);
    mav_array_memcpy(packet->id_start, id_start, sizeof(uint16_t)*25);
    mav_array_memcpy(packet->id_end, id_end, sizeof(uint16_t)*25);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FORMATION_CMD_HALF, (const char *)packet, MAVLINK_MSG_ID_FORMATION_CMD_HALF_MIN_LEN, MAVLINK_MSG_ID_FORMATION_CMD_HALF_LEN, MAVLINK_MSG_ID_FORMATION_CMD_HALF_CRC);
#endif
}
#endif

#endif

// MESSAGE FORMATION_CMD_HALF UNPACKING


/**
 * @brief Get field utc from formation_cmd_half message
 *
 * @return  Unix Timestamp in milliseconds
 */
static inline uint64_t mavlink_msg_formation_cmd_half_get_utc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field token from formation_cmd_half message
 *
 * @return   Command token
 */
static inline uint32_t mavlink_msg_formation_cmd_half_get_token(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field no_used from formation_cmd_half message
 *
 * @return  no used
 */
static inline uint16_t mavlink_msg_formation_cmd_half_get_no_used(const mavlink_message_t* msg, uint32_t *no_used)
{
    return _MAV_RETURN_uint32_t_array(msg, no_used, 4,  12);
}

/**
 * @brief Get field id_start from formation_cmd_half message
 *
 * @return   Drone ID
 */
static inline uint16_t mavlink_msg_formation_cmd_half_get_id_start(const mavlink_message_t* msg, uint16_t *id_start)
{
    return _MAV_RETURN_uint16_t_array(msg, id_start, 25,  28);
}

/**
 * @brief Get field id_end from formation_cmd_half message
 *
 * @return   Drone ID
 */
static inline uint16_t mavlink_msg_formation_cmd_half_get_id_end(const mavlink_message_t* msg, uint16_t *id_end)
{
    return _MAV_RETURN_uint16_t_array(msg, id_end, 25,  78);
}

/**
 * @brief Get field data from formation_cmd_half message
 *
 * @return   Data
 */
static inline uint16_t mavlink_msg_formation_cmd_half_get_data(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  128);
}

/**
 * @brief Get field param from formation_cmd_half message
 *
 * @return  Param
 */
static inline uint16_t mavlink_msg_formation_cmd_half_get_param(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  130);
}

/**
 * @brief Get field cmd from formation_cmd_half message
 *
 * @return   see instruction book
 */
static inline uint8_t mavlink_msg_formation_cmd_half_get_cmd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  132);
}

/**
 * @brief Get field ack from formation_cmd_half message
 *
 * @return   Ack 0-NoAck 1-Ack
 */
static inline uint8_t mavlink_msg_formation_cmd_half_get_ack(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  133);
}

/**
 * @brief Get field type from formation_cmd_half message
 *
 * @return   Broadcast or point to point 0-Broadcast 1-Point to Point
 */
static inline uint8_t mavlink_msg_formation_cmd_half_get_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  134);
}

/**
 * @brief Decode a formation_cmd_half message into a struct
 *
 * @param msg The message to decode
 * @param formation_cmd_half C-struct to decode the message contents into
 */
static inline void mavlink_msg_formation_cmd_half_decode(const mavlink_message_t* msg, mavlink_formation_cmd_half_t* formation_cmd_half)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    formation_cmd_half->utc = mavlink_msg_formation_cmd_half_get_utc(msg);
    formation_cmd_half->token = mavlink_msg_formation_cmd_half_get_token(msg);
    mavlink_msg_formation_cmd_half_get_no_used(msg, formation_cmd_half->no_used);
    mavlink_msg_formation_cmd_half_get_id_start(msg, formation_cmd_half->id_start);
    mavlink_msg_formation_cmd_half_get_id_end(msg, formation_cmd_half->id_end);
    formation_cmd_half->data = mavlink_msg_formation_cmd_half_get_data(msg);
    formation_cmd_half->param = mavlink_msg_formation_cmd_half_get_param(msg);
    formation_cmd_half->cmd = mavlink_msg_formation_cmd_half_get_cmd(msg);
    formation_cmd_half->ack = mavlink_msg_formation_cmd_half_get_ack(msg);
    formation_cmd_half->type = mavlink_msg_formation_cmd_half_get_type(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_FORMATION_CMD_HALF_LEN? msg->len : MAVLINK_MSG_ID_FORMATION_CMD_HALF_LEN;
        memset(formation_cmd_half, 0, MAVLINK_MSG_ID_FORMATION_CMD_HALF_LEN);
    memcpy(formation_cmd_half, _MAV_PAYLOAD(msg), len);
#endif
}
