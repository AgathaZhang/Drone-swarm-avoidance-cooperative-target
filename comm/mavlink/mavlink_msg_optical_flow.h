// MESSAGE OPTICAL_FLOW PACKING

#define MAVLINK_MSG_ID_OPTICAL_FLOW 100

typedef struct __mavlink_optical_flow_t
{
 uint64_t time_usec; /*< Timestamp (UNIX)*/
 uint64_t flow_timespan;
 float flow_comp_m_x; /*< Flow in meters in x-sensor direction, angular-speed compensated*/
 float flow_comp_m_y; /*< Flow in meters in y-sensor direction, angular-speed compensated*/
 float ground_distance; /*< Ground distance in meters. Positive value: distance known. Negative value: Unknown distance*/
 float flow_x; /*< Flow in pixels * 10 in x-sensor direction (dezi-pixels)*/
 float flow_y; /*< Flow in pixels * 10 in y-sensor direction (dezi-pixels)*/
 float flow_rot; 
 uint8_t sensor_id; /*< Sensor ID*/
 uint8_t quality; /*< Optical flow quality / confidence. 0: bad, 255: maximum quality*/
 uint8_t rot_quality; /*< Optical flow rotation quality / confidence. 0: bad, 255: maximum quality*/
 uint8_t hover_flag;  /*<hover flag.>*/
 uint64_t result_timespan;
}__attribute__((packed)) mavlink_optical_flow_t;

#define MAVLINK_MSG_ID_OPTICAL_FLOW_LEN 52
#define MAVLINK_MSG_ID_100_LEN 52

#define MAVLINK_MSG_ID_OPTICAL_FLOW_CRC 175
#define MAVLINK_MSG_ID_100_CRC 175



#define MAVLINK_MESSAGE_INFO_OPTICAL_FLOW { \
	"OPTICAL_FLOW", \
	13, \
	{  	\
		{ "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_optical_flow_t, time_usec) }, \
		{ "flow_timespan", NULL, MAVLINK_TYPE_UINT64_T, 8, 0, offsetof(mavlink_optical_flow_t, flow_timespan) }, \
		{ "flow_comp_m_x", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_optical_flow_t, flow_comp_m_x) }, \
		{ "flow_comp_m_y", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_optical_flow_t, flow_comp_m_y) }, \
		{ "ground_distance", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_optical_flow_t, ground_distance) }, \
		{ "flow_x", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_optical_flow_t, flow_x) }, \
		{ "flow_y", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_optical_flow_t, flow_y) }, \
		{ "flow_rot", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_optical_flow_t, flow_rot) }, \
		{ "sensor_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_optical_flow_t, sensor_id) }, \
		{ "quality", NULL, MAVLINK_TYPE_UINT8_T, 0, 41, offsetof(mavlink_optical_flow_t, quality) }, \
		{ "rot_quality", NULL, MAVLINK_TYPE_UINT8_T, 0, 42, offsetof(mavlink_optical_flow_t, rot_quality) }, \
		{ "hover_flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 43, offsetof(mavlink_optical_flow_t, hover_flag) }, \
		{ "result_timespan", NULL, MAVLINK_TYPE_UINT64_T, 0, 44, offsetof(mavlink_optical_flow_t, result_timespan) }, \
         } \
}


/**
 * @brief Pack a optical_flow message
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
static inline uint16_t mavlink_msg_optical_flow_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t time_usec, uint64_t flow_timespan, uint8_t sensor_id, int16_t flow_x, int16_t flow_y,int16_t flow_rot, float flow_comp_m_x, float flow_comp_m_y,uint8_t rot_quality, uint8_t quality,uint8_t hover_flag, float ground_distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_OPTICAL_FLOW_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint64_t(buf,8, flow_timespan);
	_mav_put_float(buf, 16, flow_comp_m_x);
	_mav_put_float(buf, 20, flow_comp_m_y);
	_mav_put_float(buf, 24, ground_distance);
	_mav_put_int16_t(buf, 28, flow_x);
	_mav_put_int16_t(buf, 32, flow_y);
	_mav_put_int16_t(buf, 36, flow_rot);
	_mav_put_uint8_t(buf, 40, sensor_id);
	_mav_put_uint8_t(buf, 41, quality);
	_mav_put_uint8_t(buf, 42, rot_quality);
	_mav_put_uint8_t(buf, 43, hover_flag);
    memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OPTICAL_FLOW_LEN);
#else
	mavlink_optical_flow_t packet;
	packet.time_usec = time_usec;
	packet.flow_timespan = flow_timespan;
	packet.flow_comp_m_x = flow_comp_m_x;
	packet.flow_comp_m_y = flow_comp_m_y;
	packet.ground_distance = ground_distance;
	packet.flow_x = flow_x;
	packet.flow_y = flow_y;
	packet.flow_rot = flow_rot;
	packet.sensor_id = sensor_id;
	packet.quality = quality;
	packet.rot_quality = rot_quality;
	packet.hover_flag = hover_flag;
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OPTICAL_FLOW_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_OPTICAL_FLOW;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_OPTICAL_FLOW_LEN,MAVLINK_MSG_ID_OPTICAL_FLOW_LEN, MAVLINK_MSG_ID_OPTICAL_FLOW_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_OPTICAL_FLOW_LEN);
#endif
}

/**
 * @brief Pack a optical_flow message on a channel
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
static inline uint16_t mavlink_msg_optical_flow_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t time_usec, uint64_t flow_timespan, uint8_t sensor_id, int16_t flow_x, int16_t flow_y,int16_t flow_rot, float flow_comp_m_x, float flow_comp_m_y,uint8_t rot_quality, uint8_t quality,uint8_t hover_flag, float ground_distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_OPTICAL_FLOW_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint64_t(buf,8, flow_timespan);
	_mav_put_float(buf, 16, flow_comp_m_x);
	_mav_put_float(buf, 20, flow_comp_m_y);
	_mav_put_float(buf, 24, ground_distance);
	_mav_put_float(buf, 28, flow_x);
	_mav_put_float(buf, 32, flow_y);
	_mav_put_float(buf, 36, flow_rot);
	_mav_put_uint8_t(buf, 40, sensor_id);
	_mav_put_uint8_t(buf, 41, quality);
	_mav_put_uint8_t(buf, 42, rot_quality);
	_mav_put_uint8_t(buf, 43, hover_flag);


        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OPTICAL_FLOW_LEN);
#else
	mavlink_optical_flow_t packet;
	packet.time_usec = time_usec;
	packet.flow_timespan = flow_timespan;
	packet.flow_comp_m_x = flow_comp_m_x;
	packet.flow_comp_m_y = flow_comp_m_y;
	packet.ground_distance = ground_distance;
	packet.flow_x = flow_x;
	packet.flow_y = flow_y;
	packet.flow_rot = flow_rot;
	packet.sensor_id = sensor_id;
	packet.quality = quality;
	packet.rot_quality = rot_quality;
	packet.hover_flag = hover_flag;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OPTICAL_FLOW_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_OPTICAL_FLOW;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_OPTICAL_FLOW_LEN, MAVLINK_MSG_ID_OPTICAL_FLOW_LEN, MAVLINK_MSG_ID_OPTICAL_FLOW_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_OPTICAL_FLOW_LEN);
#endif
}

/**
 * @brief Encode a optical_flow struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param optical_flow C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_optical_flow_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_optical_flow_t* optical_flow)
{
	return mavlink_msg_optical_flow_pack(system_id, component_id, msg, optical_flow->time_usec,optical_flow->flow_timespan, optical_flow->sensor_id, optical_flow->flow_x, optical_flow->flow_y, optical_flow->flow_rot,optical_flow->flow_comp_m_x, optical_flow->flow_comp_m_y,optical_flow->rot_quality ,optical_flow->quality,optical_flow->hover_flag, optical_flow->ground_distance);
}

/**
 * @brief Encode a optical_flow struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param optical_flow C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_optical_flow_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_optical_flow_t* optical_flow)
{
	return mavlink_msg_optical_flow_pack_chan(system_id, component_id, chan, msg, optical_flow->time_usec,optical_flow->flow_timespan, optical_flow->sensor_id, optical_flow->flow_x, optical_flow->flow_y, optical_flow->flow_rot,optical_flow->flow_comp_m_x, optical_flow->flow_comp_m_y,optical_flow->rot_quality ,optical_flow->quality,optical_flow->hover_flag, optical_flow->ground_distance);
}

/**
 * @brief Send a optical_flow message
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

static inline void mavlink_msg_optical_flow_send(mavlink_channel_t chan, uint64_t time_usec, uint64_t flow_timespan, uint8_t sensor_id, float flow_x, float flow_y,float flow_rot, float flow_comp_m_x, float flow_comp_m_y,uint8_t rot_quality, uint8_t quality,uint8_t hover_flag, float ground_distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_OPTICAL_FLOW_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint64_t(buf,8, flow_timespan);
	_mav_put_float(buf, 16, flow_comp_m_x);
	_mav_put_float(buf, 20, flow_comp_m_y);
	_mav_put_float(buf, 24, ground_distance);
	_mav_put_float(buf, 28, flow_x);
	_mav_put_float(buf, 32, flow_y);
	_mav_put_float(buf, 36, flow_rot);
	_mav_put_uint8_t(buf, 40, sensor_id);
	_mav_put_uint8_t(buf, 41, quality);
	_mav_put_uint8_t(buf, 42, rot_quality);
	_mav_put_uint8_t(buf, 43, hover_flag);



#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPTICAL_FLOW, buf, MAVLINK_MSG_ID_OPTICAL_FLOW_LEN, MAVLINK_MSG_ID_OPTICAL_FLOW_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPTICAL_FLOW, buf, MAVLINK_MSG_ID_OPTICAL_FLOW_LEN);
#endif
#else
	mavlink_optical_flow_t packet;
	packet.time_usec = time_usec;
	packet.flow_timespan = flow_timespan;
	packet.flow_comp_m_x = flow_comp_m_x;
	packet.flow_comp_m_y = flow_comp_m_y;
	packet.ground_distance = ground_distance;
	packet.flow_x = flow_x;
	packet.flow_y = flow_y;
	packet.flow_rot = flow_rot;
	packet.sensor_id = sensor_id;
	packet.quality = quality;
	packet.rot_quality = rot_quality;
	packet.hover_flag = hover_flag;


#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPTICAL_FLOW, (const char *)&packet, MAVLINK_MSG_ID_OPTICAL_FLOW_LEN, MAVLINK_MSG_ID_OPTICAL_FLOW_LEN, MAVLINK_MSG_ID_OPTICAL_FLOW_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPTICAL_FLOW, (const char *)&packet, MAVLINK_MSG_ID_OPTICAL_FLOW_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_OPTICAL_FLOW_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_optical_flow_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint64_t flow_timespan, uint8_t sensor_id, float flow_x, float flow_y,float flow_rot, float flow_comp_m_x, float flow_comp_m_y,uint8_t rot_quality, uint8_t quality,uint8_t hover_flag, float ground_distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint64_t(buf,8, flow_timespan);
	_mav_put_float(buf, 16, flow_comp_m_x);
	_mav_put_float(buf, 20, flow_comp_m_y);
	_mav_put_float(buf, 24, ground_distance);
	_mav_put_float(buf, 28, flow_x);
	_mav_put_float(buf, 32, flow_y);
	_mav_put_float(buf, 36, flow_rot);
	_mav_put_uint8_t(buf, 40, sensor_id);
	_mav_put_uint8_t(buf, 41, quality);
	_mav_put_uint8_t(buf, 42, rot_quality);
	_mav_put_uint8_t(buf, 43, hover_flag);



#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPTICAL_FLOW, buf, MAVLINK_MSG_ID_OPTICAL_FLOW_LEN, MAVLINK_MSG_ID_OPTICAL_FLOW_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPTICAL_FLOW, buf, MAVLINK_MSG_ID_OPTICAL_FLOW_LEN);
#endif
#else
	mavlink_optical_flow_t *packet = (mavlink_optical_flow_t *)msgbuf;
	packet->time_usec = time_usec;
	packet->flow_timespan = flow_timespan;
	packet->flow_comp_m_x = flow_comp_m_x;
	packet->flow_comp_m_y = flow_comp_m_y;
	packet->ground_distance = ground_distance;
	packet->flow_x = flow_x;
	packet->flow_y = flow_y;
	packet->flow_rot = flow_rot;
	packet->sensor_id = sensor_id;
	packet->quality = quality;
	packet->rot_quality = rot_quality;
	packet->hover_flag = hover_flag;


#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPTICAL_FLOW, (const char *)packet, MAVLINK_MSG_ID_OPTICAL_FLOW_LEN, MAVLINK_MSG_ID_OPTICAL_FLOW_LEN, MAVLINK_MSG_ID_OPTICAL_FLOW_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPTICAL_FLOW, (const char *)packet, MAVLINK_MSG_ID_OPTICAL_FLOW_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE OPTICAL_FLOW UNPACKING


/**
 * @brief Get field time_usec from optical_flow message
 *
 * @return Timestamp (UNIX)
 */
static inline uint64_t mavlink_msg_optical_flow_get_time_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}


/**
 * @brief Get field flow_timespan from optical_flow message
 *
 * @return flow_timespan (UNIX)
 */

static inline uint64_t mavlink_msg_optical_flow_get_flow_timespan(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  8);
}


/**
 * @brief Get field sensor_id from optical_flow message
 *
 * @return Sensor ID
 */
static inline uint8_t mavlink_msg_optical_flow_get_sensor_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  40);
}

/**
 * @brief Get field flow_x from optical_flow message
 *
 * @return Flow in pixels * 10 in x-sensor direction (dezi-pixels)
 */
static inline float mavlink_msg_optical_flow_get_flow_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field flow_y from optical_flow message
 *
 * @return Flow in pixels * 10 in y-sensor direction (dezi-pixels)
 */
static inline float mavlink_msg_optical_flow_get_flow_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}
/**
 * @brief Get field flow_rot from optical_flow message
 *
 * @return Flow in pixels * 10 in y-sensor direction (dezi-pixels)
 */

static inline float mavlink_msg_optical_flow_get_flow_rot(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  36);
}


/**
 * @brief Get field flow_comp_m_x from optical_flow message
 *
 * @return Flow in meters in x-sensor direction, angular-speed compensated
 */
static inline float mavlink_msg_optical_flow_get_flow_comp_m_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field flow_comp_m_y from optical_flow message
 *
 * @return Flow in meters in y-sensor direction, angular-speed compensated
 */
static inline float mavlink_msg_optical_flow_get_flow_comp_m_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field quality from optical_flow message
 *
 * @return Optical flow quality / confidence. 0: bad, 255: maximum quality
 */
static inline uint8_t mavlink_msg_optical_flow_get_quality(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg, 41);
}


/**
 * @brief Get field rot_quality from optical_flow message
 *
 * @return Optical flow rotation quality / confidence. 0: bad, 255: maximum quality
 */
static inline uint8_t mavlink_msg_optical_flow_get_rot_quality(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  42);
}


/**
 * @brief Get field ground_distance from optical_flow message
 *
 * @return Ground distance in meters. Positive value: distance known. Negative value: Unknown distance
 */
static inline float mavlink_msg_optical_flow_get_ground_distance(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field hover_flag from optical_flow message
 *
 * @return hover_flag
 */
static inline uint8_t mavlink_msg_optical_flow_get_hover_flag(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  43);
}

static inline uint64_t mavlink_msg_optical_flow_result_timespan(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  44);
}


/**
 * @brief Decode a optical_flow message into a struct
 *
 * @param msg The message to decode
 * @param optical_flow C-struct to decode the message contents into
 */
static inline void mavlink_msg_optical_flow_decode(mavlink_message_t* msg, mavlink_optical_flow_t* optical_flow)
{
#if MAVLINK_NEED_BYTE_SWAP

	optical_flow->time_usec = mavlink_msg_optical_flow_get_time_usec(msg);
	optical_flow->flow_timespan = mavlink_msg_optical_flow_get_flow_timespan(msg);
	optical_flow->flow_comp_m_x = mavlink_msg_optical_flow_get_flow_comp_m_x(msg);
	optical_flow->flow_comp_m_y = mavlink_msg_optical_flow_get_flow_comp_m_y(msg);
	optical_flow->ground_distance = mavlink_msg_optical_flow_get_ground_distance(msg);
	optical_flow->flow_x = mavlink_msg_optical_flow_get_flow_x(msg);
	optical_flow->flow_y = mavlink_msg_optical_flow_get_flow_y(msg);
	optical_flow->flow_rot = mavlink_msg_optical_flow_get_flow_rot(msg);
	optical_flow->sensor_id = mavlink_msg_optical_flow_get_sensor_id(msg);
	optical_flow->quality = mavlink_msg_optical_flow_get_quality(msg);
	optical_flow->rot_quality = mavlink_msg_optical_flow_get_rot_quality(msg);
	optical_flow->hover_flag = mavlink_msg_optical_flow_get_hover_flag(msg);
    optical_flow->result_timespan = mavlink_msg_optical_flow_result_timespan(msg);
#else
	memcpy(optical_flow, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_OPTICAL_FLOW_LEN);
#endif
}
