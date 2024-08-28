#pragma once
// MESSAGE FORMATION_POS_ACC PACKING

#define MAVLINK_MSG_ID_FORMATION_POS_ACC 233


typedef struct __mavlink_formation_pos_acc_t {
 float acc[3]; /*<  body acc*/
 float local_pos[3]; /*<  local acc*/
} mavlink_formation_pos_acc_t;

#define MAVLINK_MSG_ID_FORMATION_POS_ACC_LEN 24
#define MAVLINK_MSG_ID_FORMATION_POS_ACC_MIN_LEN 24
#define MAVLINK_MSG_ID_233_LEN 24
#define MAVLINK_MSG_ID_233_MIN_LEN 24

#define MAVLINK_MSG_ID_FORMATION_POS_ACC_CRC 178
#define MAVLINK_MSG_ID_233_CRC 178

#define MAVLINK_MSG_FORMATION_POS_ACC_FIELD_ACC_LEN 3
#define MAVLINK_MSG_FORMATION_POS_ACC_FIELD_LOCAL_POS_LEN 3

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_FORMATION_POS_ACC { \
    233, \
    "FORMATION_POS_ACC", \
    2, \
    {  { "acc", NULL, MAVLINK_TYPE_FLOAT, 3, 0, offsetof(mavlink_formation_pos_acc_t, acc) }, \
         { "local_pos", NULL, MAVLINK_TYPE_FLOAT, 3, 12, offsetof(mavlink_formation_pos_acc_t, local_pos) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_FORMATION_POS_ACC { \
    "FORMATION_POS_ACC", \
    2, \
    {  { "acc", NULL, MAVLINK_TYPE_FLOAT, 3, 0, offsetof(mavlink_formation_pos_acc_t, acc) }, \
         { "local_pos", NULL, MAVLINK_TYPE_FLOAT, 3, 12, offsetof(mavlink_formation_pos_acc_t, local_pos) }, \
         } \
}
#endif

/**
 * @brief Pack a formation_pos_acc message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param acc  body acc
 * @param local_pos  local acc
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_formation_pos_acc_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const float *acc, const float *local_pos)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FORMATION_POS_ACC_LEN];

    _mav_put_float_array(buf, 0, acc, 3);
    _mav_put_float_array(buf, 12, local_pos, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FORMATION_POS_ACC_LEN);
#else
    mavlink_formation_pos_acc_t packet;

    mav_array_memcpy(packet.acc, acc, sizeof(float)*3);
    mav_array_memcpy(packet.local_pos, local_pos, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FORMATION_POS_ACC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FORMATION_POS_ACC;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FORMATION_POS_ACC_MIN_LEN, MAVLINK_MSG_ID_FORMATION_POS_ACC_LEN, MAVLINK_MSG_ID_FORMATION_POS_ACC_CRC);
}

/**
 * @brief Pack a formation_pos_acc message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param acc  body acc
 * @param local_pos  local acc
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_formation_pos_acc_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const float *acc,const float *local_pos)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FORMATION_POS_ACC_LEN];

    _mav_put_float_array(buf, 0, acc, 3);
    _mav_put_float_array(buf, 12, local_pos, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FORMATION_POS_ACC_LEN);
#else
    mavlink_formation_pos_acc_t packet;

    mav_array_memcpy(packet.acc, acc, sizeof(float)*3);
    mav_array_memcpy(packet.local_pos, local_pos, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FORMATION_POS_ACC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FORMATION_POS_ACC;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FORMATION_POS_ACC_MIN_LEN, MAVLINK_MSG_ID_FORMATION_POS_ACC_LEN, MAVLINK_MSG_ID_FORMATION_POS_ACC_CRC);
}

/**
 * @brief Encode a formation_pos_acc struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param formation_pos_acc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_formation_pos_acc_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_formation_pos_acc_t* formation_pos_acc)
{
    return mavlink_msg_formation_pos_acc_pack(system_id, component_id, msg, formation_pos_acc->acc, formation_pos_acc->local_pos);
}

/**
 * @brief Encode a formation_pos_acc struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param formation_pos_acc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_formation_pos_acc_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_formation_pos_acc_t* formation_pos_acc)
{
    return mavlink_msg_formation_pos_acc_pack_chan(system_id, component_id, chan, msg, formation_pos_acc->acc, formation_pos_acc->local_pos);
}

/**
 * @brief Send a formation_pos_acc message
 * @param chan MAVLink channel to send the message
 *
 * @param acc  body acc
 * @param local_pos  local acc
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_formation_pos_acc_send(mavlink_channel_t chan, const float *acc, const float *local_pos)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FORMATION_POS_ACC_LEN];

    _mav_put_float_array(buf, 0, acc, 3);
    _mav_put_float_array(buf, 12, local_pos, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FORMATION_POS_ACC, buf, MAVLINK_MSG_ID_FORMATION_POS_ACC_MIN_LEN, MAVLINK_MSG_ID_FORMATION_POS_ACC_LEN, MAVLINK_MSG_ID_FORMATION_POS_ACC_CRC);
#else
    mavlink_formation_pos_acc_t packet;

    mav_array_memcpy(packet.acc, acc, sizeof(float)*3);
    mav_array_memcpy(packet.local_pos, local_pos, sizeof(float)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FORMATION_POS_ACC, (const char *)&packet, MAVLINK_MSG_ID_FORMATION_POS_ACC_MIN_LEN, MAVLINK_MSG_ID_FORMATION_POS_ACC_LEN, MAVLINK_MSG_ID_FORMATION_POS_ACC_CRC);
#endif
}

/**
 * @brief Send a formation_pos_acc message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_formation_pos_acc_send_struct(mavlink_channel_t chan, const mavlink_formation_pos_acc_t* formation_pos_acc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_formation_pos_acc_send(chan, formation_pos_acc->acc, formation_pos_acc->local_pos);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FORMATION_POS_ACC, (const char *)formation_pos_acc, MAVLINK_MSG_ID_FORMATION_POS_ACC_MIN_LEN, MAVLINK_MSG_ID_FORMATION_POS_ACC_LEN, MAVLINK_MSG_ID_FORMATION_POS_ACC_CRC);
#endif
}

#if MAVLINK_MSG_ID_FORMATION_POS_ACC_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_formation_pos_acc_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const float *acc, const float *local_pos)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _mav_put_float_array(buf, 0, acc, 3);
    _mav_put_float_array(buf, 12, local_pos, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FORMATION_POS_ACC, buf, MAVLINK_MSG_ID_FORMATION_POS_ACC_MIN_LEN, MAVLINK_MSG_ID_FORMATION_POS_ACC_LEN, MAVLINK_MSG_ID_FORMATION_POS_ACC_CRC);
#else
    mavlink_formation_pos_acc_t *packet = (mavlink_formation_pos_acc_t *)msgbuf;

    mav_array_memcpy(packet->acc, acc, sizeof(float)*3);
    mav_array_memcpy(packet->local_pos, local_pos, sizeof(float)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FORMATION_POS_ACC, (const char *)packet, MAVLINK_MSG_ID_FORMATION_POS_ACC_MIN_LEN, MAVLINK_MSG_ID_FORMATION_POS_ACC_LEN, MAVLINK_MSG_ID_FORMATION_POS_ACC_CRC);
#endif
}
#endif

#endif

// MESSAGE FORMATION_POS_ACC UNPACKING


/**
 * @brief Get field acc from formation_pos_acc message
 *
 * @return  body acc
 */
static inline uint16_t mavlink_msg_formation_pos_acc_get_acc(const mavlink_message_t* msg, float *acc)
{
    return _MAV_RETURN_float_array(msg, acc, 3,  0);
}

/**
 * @brief Get field local_pos from formation_pos_acc message
 *
 * @return  local acc
 */
static inline uint16_t mavlink_msg_formation_pos_acc_get_local_pos(const mavlink_message_t* msg, float *local_pos)
{
    return _MAV_RETURN_float_array(msg, local_pos, 3,  12);
}

/**
 * @brief Decode a formation_pos_acc message into a struct
 *
 * @param msg The message to decode
 * @param formation_pos_acc C-struct to decode the message contents into
 */
static inline void mavlink_msg_formation_pos_acc_decode(const mavlink_message_t* msg, mavlink_formation_pos_acc_t* formation_pos_acc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_formation_pos_acc_get_acc(msg, formation_pos_acc->acc);
    mavlink_msg_formation_pos_acc_get_local_pos(msg, formation_pos_acc->local_pos);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_FORMATION_POS_ACC_LEN? msg->len : MAVLINK_MSG_ID_FORMATION_POS_ACC_LEN;
        memset(formation_pos_acc, 0, MAVLINK_MSG_ID_FORMATION_POS_ACC_LEN);
    memcpy(formation_pos_acc, _MAV_PAYLOAD(msg), len);
#endif
}
