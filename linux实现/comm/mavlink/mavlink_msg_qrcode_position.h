#pragma once

#define MAVLINK_MSG_ID_QRCODE 114

typedef struct __mavlink_qrcode_t
{
	uint64_t time_usec; /*< Timestamp (UNIX)*/
	float roll;
	float pitch; 
	float yaw;
	float ground_distance;
	float azimuth;
	float height;
	uint8_t quality;
	uint8_t payload[10];
} __mavlink_qrcode_t;

#define MAVLINK_MSG_ID_QRCODE_LEN 43
#define MAVLINK_MSG_ID_114_LEN 43

#define MAVLINK_MSG_ID_QRCODE_CRC 175
#define MAVLINK_MSG_ID_114_CRC 175



#define MAVLINK_MESSAGE_INFO_QRCODE { \
	"QRCODE", \
	9, \
	{  	\
		{ "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(__mavlink_qrcode_t, time_usec) }, \
		{ "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(__mavlink_qrcode_t, roll) }, \
		{ "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(__mavlink_qrcode_t, pitch) }, \
		{ "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(__mavlink_qrcode_t, yaw) }, \
		{ "ground_distance", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(__mavlink_qrcode_t, ground_distance) }, \
		{ "azimuth", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(__mavlink_qrcode_t, azimuth) }, \
		{ "height", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(__mavlink_qrcode_t, height) }, \
		{ "quality", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(__mavlink_qrcode_t, quality) }, \
		{ "payload", NULL, MAVLINK_TYPE_UINT8_T, 10, 33, offsetof(__mavlink_qrcode_t, payload) }, \
         } \
}


/**
 * @brief Pack a qrcode message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (UNIX)
 * @param sensor_id Sensor ID
 * @param flow_x Flow in pixels * 10 in x-sensor direction (dezi-pixels)
 * @param flow_y Flow in pixels * 10 in y-sensor direction (dezi-pixels)
 * @param flow_comp_m_x Flow in meters in x-sensor direction, angular-speed compensated
 * @param flow_comp_m_y Flow in meters in y-sensor direction, angular-speed compensated
 * @param quality Optical flow quality / confidence. 0: bad, 255: maximum quality
 * @param ground_distance Ground distance in meters. Positive value: distance known. Negative value: Unknown distance
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_qrcode_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t time_usec, float roll, float pitch,uint8_t yaw, uint8_t quality, float ground_distance, const uint8_t *payload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_QRCODE_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_float(buf,8, roll);
	_mav_put_float(buf, 12, pitch);
	_mav_put_float(buf, 16, yaw);	
	_mav_put_float(buf, 20, ground_distance);
	_mav_put_float(buf, 24, 0);	
	_mav_put_float(buf, 28, 0);
	_mav_put_uint8_t(buf, 32, quality);
	_mav_put_uint8_t_array(buf, 33, payload, 10);
    memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_QRCODE_LEN);
#else
	__mavlink_qrcode_t packet;
	packet.time_usec = time_usec;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.ground_distance = ground_distance;
	packet.quality = quality;
	mav_array_memcpy(packet.payload, payload, sizeof(uint8_t)*10);	
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_QRCODE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_QRCODE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_QRCODE_LEN, MAVLINK_MSG_ID_QRCODE_LEN, MAVLINK_MSG_ID_QRCODE_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_QRCODE_LEN);
#endif
}

/**
 * @brief Pack a qrcode message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (UNIX)
 * @param sensor_id Sensor ID
 * @param flow_x Flow in pixels * 10 in x-sensor direction (dezi-pixels)
 * @param flow_y Flow in pixels * 10 in y-sensor direction (dezi-pixels)
 * @param flow_comp_m_x Flow in meters in x-sensor direction, angular-speed compensated
 * @param flow_comp_m_y Flow in meters in y-sensor direction, angular-speed compensated
 * @param quality Optical flow quality / confidence. 0: bad, 255: maximum quality
 * @param ground_distance Ground distance in meters. Positive value: distance known. Negative value: Unknown distance
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_qrcode_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,uint64_t time_usec, float roll, float pitch,uint8_t yaw, uint8_t quality, float ground_distance, const uint8_t *payload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_QRCODE_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_float(buf,8, roll);
	_mav_put_float(buf, 12, pitch);
	_mav_put_float(buf, 16, yaw);	
	_mav_put_float(buf, 20, ground_distance);
	_mav_put_float(buf, 24, 0);	
	_mav_put_float(buf, 28, 0);
	_mav_put_uint8_t(buf, 32, quality);
	_mav_put_uint8_t_array(buf, 33, payload, 10);


        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_QRCODE_LEN);
#else
	__mavlink_qrcode_t packet;
	packet.time_usec = time_usec;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.ground_distance = ground_distance;
	packet.quality = quality;
	mav_array_memcpy(packet.payload, payload, sizeof(uint8_t)*40);	


        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_QRCODE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_QRCODE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan,MAVLINK_MSG_ID_QRCODE_LEN, MAVLINK_MSG_ID_QRCODE_LEN, MAVLINK_MSG_ID_QRCODE_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_QRCODE_LEN);
#endif
}

/**
 * @brief Encode a qrcode struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param qrcode C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_qrcode_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const __mavlink_qrcode_t* qrcode)
{
	return mavlink_msg_qrcode_pack(system_id, component_id, msg, qrcode->time_usec,qrcode->roll, qrcode->pitch, qrcode->yaw, qrcode->ground_distance, qrcode->quality,qrcode->payload);
}

/**
 * @brief Encode a qrcode struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param qrcode C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_qrcode_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const __mavlink_qrcode_t* qrcode)
{
	return mavlink_msg_qrcode_pack_chan(system_id, component_id, chan, msg,qrcode->time_usec,qrcode->roll, qrcode->pitch, qrcode->yaw, qrcode->ground_distance, qrcode->quality,qrcode->payload);
}

/**
 * @brief Send a qrcode message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (UNIX)
 * @param sensor_id Sensor ID
 * @param flow_x Flow in pixels * 10 in x-sensor direction (dezi-pixels)
 * @param flow_y Flow in pixels * 10 in y-sensor direction (dezi-pixels)
 * @param flow_comp_m_x Flow in meters in x-sensor direction, angular-speed compensated
 * @param flow_comp_m_y Flow in meters in y-sensor direction, angular-speed compensated
 * @param quality Optical flow quality / confidence. 0: bad, 255: maximum quality
 * @param ground_distance Ground distance in meters. Positive value: distance known. Negative value: Unknown distance
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_qrcode_send(mavlink_channel_t chan,  uint64_t time_usec, float roll, float pitch,uint8_t yaw, uint8_t quality, float ground_distance, uint8_t *payload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_QRCODE_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_float(buf,8, roll);
	_mav_put_float(buf, 12, pitch);
	_mav_put_float(buf, 16, yaw);	
	_mav_put_float(buf, 20, ground_distance);
	_mav_put_float(buf, 24, 0);	
	_mav_put_float(buf, 28, 0);
	_mav_put_uint8_t(buf, 32, quality);
	_mav_put_uint8_t_array(buf, 33, payload, 10);



#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QRCODE, buf, MAVLINK_MSG_ID_QRCODE_LEN, MAVLINK_MSG_ID_QRCODE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QRCODE, buf, MAVLINK_MSG_ID_QRCODE_LEN);
#endif
#else
	__mavlink_qrcode_t packet;
	packet.time_usec = time_usec;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.ground_distance = ground_distance;
	packet.quality = quality;
	mav_array_memcpy(packet.payload, payload, sizeof(uint8_t)*10);	



#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QRCODE, (const char *)&packet,MAVLINK_MSG_ID_QRCODE_LEN, MAVLINK_MSG_ID_QRCODE_LEN, MAVLINK_MSG_ID_QRCODE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QRCODE, (const char *)&packet, MAVLINK_MSG_ID_QRCODE_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_QRCODE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_qrcode_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,   uint64_t time_usec, float roll, float pitch,uint8_t yaw, uint8_t quality, float ground_distance, uint8_t *payload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_float(buf,8, roll);
	_mav_put_float(buf, 12, pitch);
	_mav_put_float(buf, 16, yaw);	
	_mav_put_float(buf, 20, ground_distance);
	_mav_put_float(buf, 24, 0);	
	_mav_put_float(buf, 28, 0);
	_mav_put_uint8_t(buf, 32, quality);
	_mav_put_uint8_t_array(buf, 33, payload, 10);



#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QRCODE, buf, MAVLINK_MSG_ID_QRCODE_LEN, MAVLINK_MSG_ID_QRCODE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QRCODE, buf, MAVLINK_MSG_ID_QRCODE_LEN);
#endif
#else
	__mavlink_qrcode_t *packet = (__mavlink_qrcode_t *)msgbuf;
	packet->time_usec = time_usec;
	packet->roll = roll;
	packet->pitch = pitch;
	packet->yaw = yaw;
	packet->ground_distance = ground_distance;
	packet->quality = quality;
	mav_array_memcpy(packet->payload, payload, sizeof(uint8_t)*10);	



#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QRCODE, (const char *)packet, MAVLINK_MSG_ID_QRCODE_LEN, MAVLINK_MSG_ID_QRCODE_LEN, MAVLINK_MSG_ID_QRCODE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QRCODE, (const char *)packet, MAVLINK_MSG_ID_QRCODE_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE OPTICAL_FLOW UNPACKING


/**
 * @brief Get field time_usec from qrcode message
 *
 * @return Timestamp (UNIX)
 */
static inline uint64_t mavlink_msg_qrcode_get_time_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}


/**
 * @brief Get field flow_timespan from qrcode message
 *
 * @return flow_timespan (UNIX)
 */

static inline float mavlink_msg_qrcode_get_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}


/**
 * @brief Get field sensor_id from qrcode message
 *
 * @return Sensor ID
 */
static inline float mavlink_msg_qrcode_get_pitch(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field flow_x from qrcode message
 *
 * @return Flow in pixels * 10 in x-sensor direction (dezi-pixels)
 */
static inline float mavlink_msg_qrcode_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}


/**
 * @brief Get field quality from qrcode message
 *
 * @return Optical flow quality / confidence. 0: bad, 255: maximum quality
 */
static inline uint8_t mavlink_msg_qrcode_get_quality(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg, 32);
}


/**
 * @brief Get field ground_distance from qrcode message
 *
 * @return Ground distance in meters. Positive value: distance known. Negative value: Unknown distance
 */
static inline float mavlink_msg_qrcode_get_ground_distance(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}


static inline float mavlink_msg_qrcode_get_azimuth(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

static inline float mavlink_msg_qrcode_get_height(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}




/**
 * @brief Get field hover_flag from qrcode message
 *
 * @return hover_flag
 */
static inline uint8_t mavlink_msg_qrcode_get_payload(const mavlink_message_t* msg, uint8_t *payload)
{
	return _MAV_RETURN_uint8_t_array(msg, payload, 10,  33);
}




/**
 * @brief Decode a qrcode message into a struct
 *
 * @param msg The message to decode
 * @param qrcode C-struct to decode the message contents into
 */
static inline void mavlink_msg_qrcode_decode(mavlink_message_t* msg, __mavlink_qrcode_t* qrcode)
{
#if 1
	qrcode->time_usec = mavlink_msg_qrcode_get_time_usec(msg);
	qrcode->roll = mavlink_msg_qrcode_get_roll(msg);
	qrcode->pitch = mavlink_msg_qrcode_get_pitch(msg);
	qrcode->yaw = mavlink_msg_qrcode_get_yaw(msg);
	qrcode->ground_distance = mavlink_msg_qrcode_get_ground_distance(msg);
	qrcode->azimuth = mavlink_msg_qrcode_get_azimuth(msg);
	qrcode->height = mavlink_msg_qrcode_get_height(msg);
	qrcode->quality = mavlink_msg_qrcode_get_quality(msg);

    mavlink_msg_qrcode_get_payload(msg, qrcode->payload);
#else
	memcpy(qrcode, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_QRCODE_LEN);
#endif
}
