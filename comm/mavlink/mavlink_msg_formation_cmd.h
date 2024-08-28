#pragma once
// MESSAGE FORMATION_CMD PACKING

#define MAVLINK_MSG_ID_FORMATION_CMD 208


typedef struct __mavlink_formation_cmd_t {
 int32_t param3; /*<  PARAM3.*/
 int32_t param4; /*<  PARAM4.*/
 int16_t param1; /*<  PARAM1.*/
 int16_t param2; /*<  PARAM2.*/
 int16_t x; /*< [0.01m] PARAM5 / Position control of x.*/
 int16_t y; /*< [0.01m] PARAM6 / Position control of y.*/
 int16_t z; /*< [0.01m] PARAM7 / Position control of z.*/
 int16_t yaw; /*< [0.01 degree] PARAM8 / Attitude control of yaw.*/
 uint16_t start_id; /*<  start_id.*/
 uint16_t end_id; /*<  end_id.*/
 uint16_t token; /*<  Command token.*/
 uint8_t cmd; /*<  see MAV_FORMATION_CMD enum.*/
 uint8_t ack; /*<  Ack 0-NoAck 1-Ack.*/
 uint8_t type; /*<  Broadcast or point to point 0-Broadcast 1-Point to Point.*/
} mavlink_formation_cmd_t;

#define MAVLINK_MSG_ID_FORMATION_CMD_LEN 29
#define MAVLINK_MSG_ID_FORMATION_CMD_MIN_LEN 29
#define MAVLINK_MSG_ID_208_LEN 29
#define MAVLINK_MSG_ID_208_MIN_LEN 29

#define MAVLINK_MSG_ID_FORMATION_CMD_CRC 118
#define MAVLINK_MSG_ID_208_CRC 118



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_FORMATION_CMD { \
    208, \
    "FORMATION_CMD", \
    14, \
    {  { "param1", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_formation_cmd_t, param1) }, \
         { "param2", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_formation_cmd_t, param2) }, \
         { "param3", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_formation_cmd_t, param3) }, \
         { "param4", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_formation_cmd_t, param4) }, \
         { "x", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_formation_cmd_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_formation_cmd_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_formation_cmd_t, z) }, \
         { "yaw", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_formation_cmd_t, yaw) }, \
         { "start_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_formation_cmd_t, start_id) }, \
         { "end_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_formation_cmd_t, end_id) }, \
         { "token", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_formation_cmd_t, token) }, \
         { "cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_formation_cmd_t, cmd) }, \
         { "ack", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_formation_cmd_t, ack) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_formation_cmd_t, type) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_FORMATION_CMD { \
    "FORMATION_CMD", \
    14, \
    {  { "param1", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_formation_cmd_t, param1) }, \
         { "param2", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_formation_cmd_t, param2) }, \
         { "param3", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_formation_cmd_t, param3) }, \
         { "param4", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_formation_cmd_t, param4) }, \
         { "x", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_formation_cmd_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_formation_cmd_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_formation_cmd_t, z) }, \
         { "yaw", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_formation_cmd_t, yaw) }, \
         { "start_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_formation_cmd_t, start_id) }, \
         { "end_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_formation_cmd_t, end_id) }, \
         { "token", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_formation_cmd_t, token) }, \
         { "cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_formation_cmd_t, cmd) }, \
         { "ack", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_formation_cmd_t, ack) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_formation_cmd_t, type) }, \
         } \
}
#endif

/**
 * @brief Pack a formation_cmd message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param param1  PARAM1.
 * @param param2  PARAM2.
 * @param param3  PARAM3.
 * @param param4  PARAM4.
 * @param x [0.01m] PARAM5 / Position control of x.
 * @param y [0.01m] PARAM6 / Position control of y.
 * @param z [0.01m] PARAM7 / Position control of z.
 * @param yaw [0.01 degree] PARAM8 / Attitude control of yaw.
 * @param start_id  start_id.
 * @param end_id  end_id.
 * @param token  Command token.
 * @param cmd  see MAV_FORMATION_CMD enum.
 * @param ack  Ack 0-NoAck 1-Ack.
 * @param type  Broadcast or point to point 0-Broadcast 1-Point to Point.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_formation_cmd_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int16_t param1, int16_t param2, int32_t param3, int32_t param4, int16_t x, int16_t y, int16_t z, int16_t yaw, uint16_t start_id, uint16_t end_id, uint16_t token, uint8_t cmd, uint8_t ack, uint8_t type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FORMATION_CMD_LEN];
    _mav_put_int32_t(buf, 0, param3);
    _mav_put_int32_t(buf, 4, param4);
    _mav_put_int16_t(buf, 8, param1);
    _mav_put_int16_t(buf, 10, param2);
    _mav_put_int16_t(buf, 12, x);
    _mav_put_int16_t(buf, 14, y);
    _mav_put_int16_t(buf, 16, z);
    _mav_put_int16_t(buf, 18, yaw);
    _mav_put_uint16_t(buf, 20, start_id);
    _mav_put_uint16_t(buf, 22, end_id);
    _mav_put_uint16_t(buf, 24, token);
    _mav_put_uint8_t(buf, 26, cmd);
    _mav_put_uint8_t(buf, 27, ack);
    _mav_put_uint8_t(buf, 28, type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FORMATION_CMD_LEN);
#else
    mavlink_formation_cmd_t packet;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.yaw = yaw;
    packet.start_id = start_id;
    packet.end_id = end_id;
    packet.token = token;
    packet.cmd = cmd;
    packet.ack = ack;
    packet.type = type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FORMATION_CMD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FORMATION_CMD;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FORMATION_CMD_MIN_LEN, MAVLINK_MSG_ID_FORMATION_CMD_LEN, MAVLINK_MSG_ID_FORMATION_CMD_CRC);
}

/**
 * @brief Pack a formation_cmd message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param param1  PARAM1.
 * @param param2  PARAM2.
 * @param param3  PARAM3.
 * @param param4  PARAM4.
 * @param x [0.01m] PARAM5 / Position control of x.
 * @param y [0.01m] PARAM6 / Position control of y.
 * @param z [0.01m] PARAM7 / Position control of z.
 * @param yaw [0.01 degree] PARAM8 / Attitude control of yaw.
 * @param start_id  start_id.
 * @param end_id  end_id.
 * @param token  Command token.
 * @param cmd  see MAV_FORMATION_CMD enum.
 * @param ack  Ack 0-NoAck 1-Ack.
 * @param type  Broadcast or point to point 0-Broadcast 1-Point to Point.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_formation_cmd_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int16_t param1,int16_t param2,int32_t param3,int32_t param4,int16_t x,int16_t y,int16_t z,int16_t yaw,uint16_t start_id,uint16_t end_id,uint16_t token,uint8_t cmd,uint8_t ack,uint8_t type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FORMATION_CMD_LEN];
    _mav_put_int32_t(buf, 0, param3);
    _mav_put_int32_t(buf, 4, param4);
    _mav_put_int16_t(buf, 8, param1);
    _mav_put_int16_t(buf, 10, param2);
    _mav_put_int16_t(buf, 12, x);
    _mav_put_int16_t(buf, 14, y);
    _mav_put_int16_t(buf, 16, z);
    _mav_put_int16_t(buf, 18, yaw);
    _mav_put_uint16_t(buf, 20, start_id);
    _mav_put_uint16_t(buf, 22, end_id);
    _mav_put_uint16_t(buf, 24, token);
    _mav_put_uint8_t(buf, 26, cmd);
    _mav_put_uint8_t(buf, 27, ack);
    _mav_put_uint8_t(buf, 28, type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FORMATION_CMD_LEN);
#else
    mavlink_formation_cmd_t packet;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.yaw = yaw;
    packet.start_id = start_id;
    packet.end_id = end_id;
    packet.token = token;
    packet.cmd = cmd;
    packet.ack = ack;
    packet.type = type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FORMATION_CMD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FORMATION_CMD;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FORMATION_CMD_MIN_LEN, MAVLINK_MSG_ID_FORMATION_CMD_LEN, MAVLINK_MSG_ID_FORMATION_CMD_CRC);
}

/**
 * @brief Encode a formation_cmd struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param formation_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_formation_cmd_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_formation_cmd_t* formation_cmd)
{
    return mavlink_msg_formation_cmd_pack(system_id, component_id, msg, formation_cmd->param1, formation_cmd->param2, formation_cmd->param3, formation_cmd->param4, formation_cmd->x, formation_cmd->y, formation_cmd->z, formation_cmd->yaw, formation_cmd->start_id, formation_cmd->end_id, formation_cmd->token, formation_cmd->cmd, formation_cmd->ack, formation_cmd->type);
}

/**
 * @brief Encode a formation_cmd struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param formation_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_formation_cmd_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_formation_cmd_t* formation_cmd)
{
    return mavlink_msg_formation_cmd_pack_chan(system_id, component_id, chan, msg, formation_cmd->param1, formation_cmd->param2, formation_cmd->param3, formation_cmd->param4, formation_cmd->x, formation_cmd->y, formation_cmd->z, formation_cmd->yaw, formation_cmd->start_id, formation_cmd->end_id, formation_cmd->token, formation_cmd->cmd, formation_cmd->ack, formation_cmd->type);
}

/**
 * @brief Send a formation_cmd message
 * @param chan MAVLink channel to send the message
 *
 * @param param1  PARAM1.
 * @param param2  PARAM2.
 * @param param3  PARAM3.
 * @param param4  PARAM4.
 * @param x [0.01m] PARAM5 / Position control of x.
 * @param y [0.01m] PARAM6 / Position control of y.
 * @param z [0.01m] PARAM7 / Position control of z.
 * @param yaw [0.01 degree] PARAM8 / Attitude control of yaw.
 * @param start_id  start_id.
 * @param end_id  end_id.
 * @param token  Command token.
 * @param cmd  see MAV_FORMATION_CMD enum.
 * @param ack  Ack 0-NoAck 1-Ack.
 * @param type  Broadcast or point to point 0-Broadcast 1-Point to Point.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_formation_cmd_send(mavlink_channel_t chan, int16_t param1, int16_t param2, int32_t param3, int32_t param4, int16_t x, int16_t y, int16_t z, int16_t yaw, uint16_t start_id, uint16_t end_id, uint16_t token, uint8_t cmd, uint8_t ack, uint8_t type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FORMATION_CMD_LEN];
    _mav_put_int32_t(buf, 0, param3);
    _mav_put_int32_t(buf, 4, param4);
    _mav_put_int16_t(buf, 8, param1);
    _mav_put_int16_t(buf, 10, param2);
    _mav_put_int16_t(buf, 12, x);
    _mav_put_int16_t(buf, 14, y);
    _mav_put_int16_t(buf, 16, z);
    _mav_put_int16_t(buf, 18, yaw);
    _mav_put_uint16_t(buf, 20, start_id);
    _mav_put_uint16_t(buf, 22, end_id);
    _mav_put_uint16_t(buf, 24, token);
    _mav_put_uint8_t(buf, 26, cmd);
    _mav_put_uint8_t(buf, 27, ack);
    _mav_put_uint8_t(buf, 28, type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FORMATION_CMD, buf, MAVLINK_MSG_ID_FORMATION_CMD_MIN_LEN, MAVLINK_MSG_ID_FORMATION_CMD_LEN, MAVLINK_MSG_ID_FORMATION_CMD_CRC);
#else
    mavlink_formation_cmd_t packet;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.yaw = yaw;
    packet.start_id = start_id;
    packet.end_id = end_id;
    packet.token = token;
    packet.cmd = cmd;
    packet.ack = ack;
    packet.type = type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FORMATION_CMD, (const char *)&packet, MAVLINK_MSG_ID_FORMATION_CMD_MIN_LEN, MAVLINK_MSG_ID_FORMATION_CMD_LEN, MAVLINK_MSG_ID_FORMATION_CMD_CRC);
#endif
}

/**
 * @brief Send a formation_cmd message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_formation_cmd_send_struct(mavlink_channel_t chan, const mavlink_formation_cmd_t* formation_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_formation_cmd_send(chan, formation_cmd->param1, formation_cmd->param2, formation_cmd->param3, formation_cmd->param4, formation_cmd->x, formation_cmd->y, formation_cmd->z, formation_cmd->yaw, formation_cmd->start_id, formation_cmd->end_id, formation_cmd->token, formation_cmd->cmd, formation_cmd->ack, formation_cmd->type);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FORMATION_CMD, (const char *)formation_cmd, MAVLINK_MSG_ID_FORMATION_CMD_MIN_LEN, MAVLINK_MSG_ID_FORMATION_CMD_LEN, MAVLINK_MSG_ID_FORMATION_CMD_CRC);
#endif
}

#if MAVLINK_MSG_ID_FORMATION_CMD_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_formation_cmd_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int16_t param1, int16_t param2, int32_t param3, int32_t param4, int16_t x, int16_t y, int16_t z, int16_t yaw, uint16_t start_id, uint16_t end_id, uint16_t token, uint8_t cmd, uint8_t ack, uint8_t type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, param3);
    _mav_put_int32_t(buf, 4, param4);
    _mav_put_int16_t(buf, 8, param1);
    _mav_put_int16_t(buf, 10, param2);
    _mav_put_int16_t(buf, 12, x);
    _mav_put_int16_t(buf, 14, y);
    _mav_put_int16_t(buf, 16, z);
    _mav_put_int16_t(buf, 18, yaw);
    _mav_put_uint16_t(buf, 20, start_id);
    _mav_put_uint16_t(buf, 22, end_id);
    _mav_put_uint16_t(buf, 24, token);
    _mav_put_uint8_t(buf, 26, cmd);
    _mav_put_uint8_t(buf, 27, ack);
    _mav_put_uint8_t(buf, 28, type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FORMATION_CMD, buf, MAVLINK_MSG_ID_FORMATION_CMD_MIN_LEN, MAVLINK_MSG_ID_FORMATION_CMD_LEN, MAVLINK_MSG_ID_FORMATION_CMD_CRC);
#else
    mavlink_formation_cmd_t *packet = (mavlink_formation_cmd_t *)msgbuf;
    packet->param3 = param3;
    packet->param4 = param4;
    packet->param1 = param1;
    packet->param2 = param2;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->yaw = yaw;
    packet->start_id = start_id;
    packet->end_id = end_id;
    packet->token = token;
    packet->cmd = cmd;
    packet->ack = ack;
    packet->type = type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FORMATION_CMD, (const char *)packet, MAVLINK_MSG_ID_FORMATION_CMD_MIN_LEN, MAVLINK_MSG_ID_FORMATION_CMD_LEN, MAVLINK_MSG_ID_FORMATION_CMD_CRC);
#endif
}
#endif

#endif

// MESSAGE FORMATION_CMD UNPACKING


/**
 * @brief Get field param1 from formation_cmd message
 *
 * @return  PARAM1.
 */
static inline int16_t mavlink_msg_formation_cmd_get_param1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Get field param2 from formation_cmd message
 *
 * @return  PARAM2.
 */
static inline int16_t mavlink_msg_formation_cmd_get_param2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  10);
}

/**
 * @brief Get field param3 from formation_cmd message
 *
 * @return  PARAM3.
 */
static inline int32_t mavlink_msg_formation_cmd_get_param3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field param4 from formation_cmd message
 *
 * @return  PARAM4.
 */
static inline int32_t mavlink_msg_formation_cmd_get_param4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field x from formation_cmd message
 *
 * @return [0.01m] PARAM5 / Position control of x.
 */
static inline int16_t mavlink_msg_formation_cmd_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  12);
}

/**
 * @brief Get field y from formation_cmd message
 *
 * @return [0.01m] PARAM6 / Position control of y.
 */
static inline int16_t mavlink_msg_formation_cmd_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  14);
}

/**
 * @brief Get field z from formation_cmd message
 *
 * @return [0.01m] PARAM7 / Position control of z.
 */
static inline int16_t mavlink_msg_formation_cmd_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  16);
}

/**
 * @brief Get field yaw from formation_cmd message
 *
 * @return [0.01 degree] PARAM8 / Attitude control of yaw.
 */
static inline int16_t mavlink_msg_formation_cmd_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  18);
}

/**
 * @brief Get field start_id from formation_cmd message
 *
 * @return  start_id.
 */
static inline uint16_t mavlink_msg_formation_cmd_get_start_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Get field end_id from formation_cmd message
 *
 * @return  end_id.
 */
static inline uint16_t mavlink_msg_formation_cmd_get_end_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  22);
}

/**
 * @brief Get field token from formation_cmd message
 *
 * @return  Command token.
 */
static inline uint16_t mavlink_msg_formation_cmd_get_token(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  24);
}

/**
 * @brief Get field cmd from formation_cmd message
 *
 * @return  see MAV_FORMATION_CMD enum.
 */
static inline uint8_t mavlink_msg_formation_cmd_get_cmd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  26);
}

/**
 * @brief Get field ack from formation_cmd message
 *
 * @return  Ack 0-NoAck 1-Ack.
 */
static inline uint8_t mavlink_msg_formation_cmd_get_ack(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  27);
}

/**
 * @brief Get field type from formation_cmd message
 *
 * @return  Broadcast or point to point 0-Broadcast 1-Point to Point.
 */
static inline uint8_t mavlink_msg_formation_cmd_get_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Decode a formation_cmd message into a struct
 *
 * @param msg The message to decode
 * @param formation_cmd C-struct to decode the message contents into
 */
static inline void mavlink_msg_formation_cmd_decode(const mavlink_message_t* msg, mavlink_formation_cmd_t* formation_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    formation_cmd->param3 = mavlink_msg_formation_cmd_get_param3(msg);
    formation_cmd->param4 = mavlink_msg_formation_cmd_get_param4(msg);
    formation_cmd->param1 = mavlink_msg_formation_cmd_get_param1(msg);
    formation_cmd->param2 = mavlink_msg_formation_cmd_get_param2(msg);
    formation_cmd->x = mavlink_msg_formation_cmd_get_x(msg);
    formation_cmd->y = mavlink_msg_formation_cmd_get_y(msg);
    formation_cmd->z = mavlink_msg_formation_cmd_get_z(msg);
    formation_cmd->yaw = mavlink_msg_formation_cmd_get_yaw(msg);
    formation_cmd->start_id = mavlink_msg_formation_cmd_get_start_id(msg);
    formation_cmd->end_id = mavlink_msg_formation_cmd_get_end_id(msg);
    formation_cmd->token = mavlink_msg_formation_cmd_get_token(msg);
    formation_cmd->cmd = mavlink_msg_formation_cmd_get_cmd(msg);
    formation_cmd->ack = mavlink_msg_formation_cmd_get_ack(msg);
    formation_cmd->type = mavlink_msg_formation_cmd_get_type(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_FORMATION_CMD_LEN? msg->len : MAVLINK_MSG_ID_FORMATION_CMD_LEN;
        memset(formation_cmd, 0, MAVLINK_MSG_ID_FORMATION_CMD_LEN);
    memcpy(formation_cmd, _MAV_PAYLOAD(msg), len);
#endif
}
