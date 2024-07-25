#pragma once
// MESSAGE MOV_POS_CMD PACKING

#define MAVLINK_MSG_ID_MOV_POS_CMD 201


typedef struct __mavlink_mov_pos_cmd_t {
 uint32_t token; /*<  Command token*/
 int32_t pos_data[3]; /*<  set position data*/
 float yaw; /*<  set yaw data*/
 uint16_t drone_id; /*<  Drone ID*/
 uint8_t cmd; /*<   see instruction book*/
 uint8_t ack; /*<   Ack 0-NoAck 1-Ack*/
 uint8_t reserve[8]; /*<   reserve variable*/
} mavlink_mov_pos_cmd_t;

#define MAVLINK_MSG_ID_MOV_POS_CMD_LEN 32
#define MAVLINK_MSG_ID_MOV_POS_CMD_MIN_LEN 32
#define MAVLINK_MSG_ID_201_LEN 32
#define MAVLINK_MSG_ID_201_MIN_LEN 32

#define MAVLINK_MSG_ID_MOV_POS_CMD_CRC 87
#define MAVLINK_MSG_ID_201_CRC 87

#define MAVLINK_MSG_MOV_POS_CMD_FIELD_POS_DATA_LEN 3
#define MAVLINK_MSG_MOV_POS_CMD_FIELD_RESERVE_LEN 8

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MOV_POS_CMD { \
    201, \
    "MOV_POS_CMD", \
    7, \
    {  { "token", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_mov_pos_cmd_t, token) }, \
         { "drone_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_mov_pos_cmd_t, drone_id) }, \
         { "pos_data", NULL, MAVLINK_TYPE_INT32_T, 3, 4, offsetof(mavlink_mov_pos_cmd_t, pos_data) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_mov_pos_cmd_t, yaw) }, \
         { "cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_mov_pos_cmd_t, cmd) }, \
         { "ack", NULL, MAVLINK_TYPE_UINT8_T, 0, 23, offsetof(mavlink_mov_pos_cmd_t, ack) }, \
         { "reserve", NULL, MAVLINK_TYPE_UINT8_T, 8, 24, offsetof(mavlink_mov_pos_cmd_t, reserve) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MOV_POS_CMD { \
    "MOV_POS_CMD", \
    7, \
    {  { "token", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_mov_pos_cmd_t, token) }, \
         { "drone_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_mov_pos_cmd_t, drone_id) }, \
         { "pos_data", NULL, MAVLINK_TYPE_INT32_T, 3, 4, offsetof(mavlink_mov_pos_cmd_t, pos_data) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_mov_pos_cmd_t, yaw) }, \
         { "cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_mov_pos_cmd_t, cmd) }, \
         { "ack", NULL, MAVLINK_TYPE_UINT8_T, 0, 23, offsetof(mavlink_mov_pos_cmd_t, ack) }, \
         { "reserve", NULL, MAVLINK_TYPE_UINT8_T, 8, 24, offsetof(mavlink_mov_pos_cmd_t, reserve) }, \
         } \
}
#endif

/**
 * @brief Pack a mov_pos_cmd message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param token  Command token
 * @param drone_id  Drone ID
 * @param pos_data  set position data
 * @param yaw  set yaw data
 * @param cmd   see instruction book
 * @param ack   Ack 0-NoAck 1-Ack
 * @param reserve   reserve variable
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mov_pos_cmd_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t token, uint16_t drone_id, const int32_t *pos_data, float yaw, uint8_t cmd, uint8_t ack, const uint8_t *reserve)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOV_POS_CMD_LEN];
    _mav_put_uint32_t(buf, 0, token);
    _mav_put_float(buf, 16, yaw);
    _mav_put_uint16_t(buf, 20, drone_id);
    _mav_put_uint8_t(buf, 22, cmd);
    _mav_put_uint8_t(buf, 23, ack);
    _mav_put_int32_t_array(buf, 4, pos_data, 3);
    _mav_put_uint8_t_array(buf, 24, reserve, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOV_POS_CMD_LEN);
#else
    mavlink_mov_pos_cmd_t packet;
    packet.token = token;
    packet.yaw = yaw;
    packet.drone_id = drone_id;
    packet.cmd = cmd;
    packet.ack = ack;
    mav_array_memcpy(packet.pos_data, pos_data, sizeof(int32_t)*3);
    mav_array_memcpy(packet.reserve, reserve, sizeof(uint8_t)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOV_POS_CMD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOV_POS_CMD;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MOV_POS_CMD_MIN_LEN, MAVLINK_MSG_ID_MOV_POS_CMD_LEN, MAVLINK_MSG_ID_MOV_POS_CMD_CRC);
}

/**
 * @brief Pack a mov_pos_cmd message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param token  Command token
 * @param drone_id  Drone ID
 * @param pos_data  set position data
 * @param yaw  set yaw data
 * @param cmd   see instruction book
 * @param ack   Ack 0-NoAck 1-Ack
 * @param reserve   reserve variable
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mov_pos_cmd_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t token,uint16_t drone_id,const int32_t *pos_data,float yaw,uint8_t cmd,uint8_t ack,const uint8_t *reserve)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOV_POS_CMD_LEN];
    _mav_put_uint32_t(buf, 0, token);
    _mav_put_float(buf, 16, yaw);
    _mav_put_uint16_t(buf, 20, drone_id);
    _mav_put_uint8_t(buf, 22, cmd);
    _mav_put_uint8_t(buf, 23, ack);
    _mav_put_int32_t_array(buf, 4, pos_data, 3);
    _mav_put_uint8_t_array(buf, 24, reserve, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOV_POS_CMD_LEN);
#else
    mavlink_mov_pos_cmd_t packet;
    packet.token = token;
    packet.yaw = yaw;
    packet.drone_id = drone_id;
    packet.cmd = cmd;
    packet.ack = ack;
    mav_array_memcpy(packet.pos_data, pos_data, sizeof(int32_t)*3);
    mav_array_memcpy(packet.reserve, reserve, sizeof(uint8_t)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOV_POS_CMD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOV_POS_CMD;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MOV_POS_CMD_MIN_LEN, MAVLINK_MSG_ID_MOV_POS_CMD_LEN, MAVLINK_MSG_ID_MOV_POS_CMD_CRC);
}

/**
 * @brief Encode a mov_pos_cmd struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mov_pos_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mov_pos_cmd_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mov_pos_cmd_t* mov_pos_cmd)
{
    return mavlink_msg_mov_pos_cmd_pack(system_id, component_id, msg, mov_pos_cmd->token, mov_pos_cmd->drone_id, mov_pos_cmd->pos_data, mov_pos_cmd->yaw, mov_pos_cmd->cmd, mov_pos_cmd->ack, mov_pos_cmd->reserve);
}

/**
 * @brief Encode a mov_pos_cmd struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mov_pos_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mov_pos_cmd_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mov_pos_cmd_t* mov_pos_cmd)
{
    return mavlink_msg_mov_pos_cmd_pack_chan(system_id, component_id, chan, msg, mov_pos_cmd->token, mov_pos_cmd->drone_id, mov_pos_cmd->pos_data, mov_pos_cmd->yaw, mov_pos_cmd->cmd, mov_pos_cmd->ack, mov_pos_cmd->reserve);
}

/**
 * @brief Send a mov_pos_cmd message
 * @param chan MAVLink channel to send the message
 *
 * @param token  Command token
 * @param drone_id  Drone ID
 * @param pos_data  set position data
 * @param yaw  set yaw data
 * @param cmd   see instruction book
 * @param ack   Ack 0-NoAck 1-Ack
 * @param reserve   reserve variable
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mov_pos_cmd_send(mavlink_channel_t chan, uint32_t token, uint16_t drone_id, const int32_t *pos_data, float yaw, uint8_t cmd, uint8_t ack, const uint8_t *reserve)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOV_POS_CMD_LEN];
    _mav_put_uint32_t(buf, 0, token);
    _mav_put_float(buf, 16, yaw);
    _mav_put_uint16_t(buf, 20, drone_id);
    _mav_put_uint8_t(buf, 22, cmd);
    _mav_put_uint8_t(buf, 23, ack);
    _mav_put_int32_t_array(buf, 4, pos_data, 3);
    _mav_put_uint8_t_array(buf, 24, reserve, 8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOV_POS_CMD, buf, MAVLINK_MSG_ID_MOV_POS_CMD_MIN_LEN, MAVLINK_MSG_ID_MOV_POS_CMD_LEN, MAVLINK_MSG_ID_MOV_POS_CMD_CRC);
#else
    mavlink_mov_pos_cmd_t packet;
    packet.token = token;
    packet.yaw = yaw;
    packet.drone_id = drone_id;
    packet.cmd = cmd;
    packet.ack = ack;
    mav_array_memcpy(packet.pos_data, pos_data, sizeof(int32_t)*3);
    mav_array_memcpy(packet.reserve, reserve, sizeof(uint8_t)*8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOV_POS_CMD, (const char *)&packet, MAVLINK_MSG_ID_MOV_POS_CMD_MIN_LEN, MAVLINK_MSG_ID_MOV_POS_CMD_LEN, MAVLINK_MSG_ID_MOV_POS_CMD_CRC);
#endif
}

/**
 * @brief Send a mov_pos_cmd message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mov_pos_cmd_send_struct(mavlink_channel_t chan, const mavlink_mov_pos_cmd_t* mov_pos_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mov_pos_cmd_send(chan, mov_pos_cmd->token, mov_pos_cmd->drone_id, mov_pos_cmd->pos_data, mov_pos_cmd->yaw, mov_pos_cmd->cmd, mov_pos_cmd->ack, mov_pos_cmd->reserve);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOV_POS_CMD, (const char *)mov_pos_cmd, MAVLINK_MSG_ID_MOV_POS_CMD_MIN_LEN, MAVLINK_MSG_ID_MOV_POS_CMD_LEN, MAVLINK_MSG_ID_MOV_POS_CMD_CRC);
#endif
}

#if MAVLINK_MSG_ID_MOV_POS_CMD_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mov_pos_cmd_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t token, uint16_t drone_id, const int32_t *pos_data, float yaw, uint8_t cmd, uint8_t ack, const uint8_t *reserve)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, token);
    _mav_put_float(buf, 16, yaw);
    _mav_put_uint16_t(buf, 20, drone_id);
    _mav_put_uint8_t(buf, 22, cmd);
    _mav_put_uint8_t(buf, 23, ack);
    _mav_put_int32_t_array(buf, 4, pos_data, 3);
    _mav_put_uint8_t_array(buf, 24, reserve, 8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOV_POS_CMD, buf, MAVLINK_MSG_ID_MOV_POS_CMD_MIN_LEN, MAVLINK_MSG_ID_MOV_POS_CMD_LEN, MAVLINK_MSG_ID_MOV_POS_CMD_CRC);
#else
    mavlink_mov_pos_cmd_t *packet = (mavlink_mov_pos_cmd_t *)msgbuf;
    packet->token = token;
    packet->yaw = yaw;
    packet->drone_id = drone_id;
    packet->cmd = cmd;
    packet->ack = ack;
    mav_array_memcpy(packet->pos_data, pos_data, sizeof(int32_t)*3);
    mav_array_memcpy(packet->reserve, reserve, sizeof(uint8_t)*8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOV_POS_CMD, (const char *)packet, MAVLINK_MSG_ID_MOV_POS_CMD_MIN_LEN, MAVLINK_MSG_ID_MOV_POS_CMD_LEN, MAVLINK_MSG_ID_MOV_POS_CMD_CRC);
#endif
}
#endif

#endif

// MESSAGE MOV_POS_CMD UNPACKING


/**
 * @brief Get field token from mov_pos_cmd message
 *
 * @return  Command token
 */
static inline uint32_t mavlink_msg_mov_pos_cmd_get_token(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field drone_id from mov_pos_cmd message
 *
 * @return  Drone ID
 */
static inline uint16_t mavlink_msg_mov_pos_cmd_get_drone_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Get field pos_data from mov_pos_cmd message
 *
 * @return  set position data
 */
static inline uint16_t mavlink_msg_mov_pos_cmd_get_pos_data(const mavlink_message_t* msg, int32_t *pos_data)
{
    return _MAV_RETURN_int32_t_array(msg, pos_data, 3,  4);
}

/**
 * @brief Get field yaw from mov_pos_cmd message
 *
 * @return  set yaw data
 */
static inline float mavlink_msg_mov_pos_cmd_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field cmd from mov_pos_cmd message
 *
 * @return   see instruction book
 */
static inline uint8_t mavlink_msg_mov_pos_cmd_get_cmd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  22);
}

/**
 * @brief Get field ack from mov_pos_cmd message
 *
 * @return   Ack 0-NoAck 1-Ack
 */
static inline uint8_t mavlink_msg_mov_pos_cmd_get_ack(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  23);
}

/**
 * @brief Get field reserve from mov_pos_cmd message
 *
 * @return   reserve variable
 */
static inline uint16_t mavlink_msg_mov_pos_cmd_get_reserve(const mavlink_message_t* msg, uint8_t *reserve)
{
    return _MAV_RETURN_uint8_t_array(msg, reserve, 8,  24);
}

/**
 * @brief Decode a mov_pos_cmd message into a struct
 *
 * @param msg The message to decode
 * @param mov_pos_cmd C-struct to decode the message contents into
 */
static inline void mavlink_msg_mov_pos_cmd_decode(const mavlink_message_t* msg, mavlink_mov_pos_cmd_t* mov_pos_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mov_pos_cmd->token = mavlink_msg_mov_pos_cmd_get_token(msg);
    mavlink_msg_mov_pos_cmd_get_pos_data(msg, mov_pos_cmd->pos_data);
    mov_pos_cmd->yaw = mavlink_msg_mov_pos_cmd_get_yaw(msg);
    mov_pos_cmd->drone_id = mavlink_msg_mov_pos_cmd_get_drone_id(msg);
    mov_pos_cmd->cmd = mavlink_msg_mov_pos_cmd_get_cmd(msg);
    mov_pos_cmd->ack = mavlink_msg_mov_pos_cmd_get_ack(msg);
    mavlink_msg_mov_pos_cmd_get_reserve(msg, mov_pos_cmd->reserve);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MOV_POS_CMD_LEN? msg->len : MAVLINK_MSG_ID_MOV_POS_CMD_LEN;
        memset(mov_pos_cmd, 0, MAVLINK_MSG_ID_MOV_POS_CMD_LEN);
    memcpy(mov_pos_cmd, _MAV_PAYLOAD(msg), len);
#endif
}
