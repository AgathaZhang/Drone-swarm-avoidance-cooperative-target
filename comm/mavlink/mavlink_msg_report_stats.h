#pragma once
// MESSAGE REPORT_STATS PACKING

#define MAVLINK_MSG_ID_REPORT_STATS 207


typedef struct __mavlink_report_stats_t {
 uint64_t utc; /*<  Drone System Time, ms.*/
 uint64_t flight_time; /*<  Flight time, unit:ms.*/
 int32_t lat; /*<  Position X, unit:cm.*/
 int32_t lon; /*<  Position Y, unit:cm.*/
 int16_t alt; /*<  Position Z, unit:cm.*/
 int16_t x; /*<  Formation Position X, unit:cm.*/
 int16_t y; /*<  Formation Position Y, unit:cm.*/
 int16_t z; /*<  Formation Position Z, unit:cm.*/
 uint16_t yaw; /*<  Yaw, unit:0.01 degree.*/
 uint16_t sensors_present; /*<  Sensor Present.*/
 uint16_t sensors_health; /*<  Sensor Healthy.*/
 uint16_t drone_id; /*<  Drone ID.*/
 uint16_t firmware_version; /*<  Version of firmware, 1234 means 1.2.3.4.*/
 uint16_t system_version; /*<  Version of system, 1234 means 1.2.3.4.*/
 uint16_t dance_version; /*<  Version of dance, 123 means 1.2.3.*/
 uint8_t dance_name[8]; /*<  Dance Name.*/
 uint8_t dance_md5[16]; /*<  Dance Md5.*/
 uint8_t product_id[16]; /*<  Product ID.*/
 uint8_t aux_token; /*<  Aux dance step token.*/
 uint8_t time_token; /*<  Time Synchronization token.*/
 uint8_t battery_volumn; /*<  Battery volumn.*/
 uint8_t formation_status; /*<  Formation status.*/
 uint8_t rgb_status; /*<  RGB status.*/
 uint8_t acc_clibration_status; /*<  Acc Clibration status.*/
 uint8_t mag_clibration_status; /*<  Mag Clibration status.*/
 int8_t temperature; /*<  Temperature, unit:Centigrade.*/
 uint8_t block_status; /*<  Block Programming status.*/
 uint8_t drone_status; /*<  Drone flight status, 0:no fly,1:flying.*/
} mavlink_report_stats_t;

#define MAVLINK_MSG_ID_REPORT_STATS_LEN 96
#define MAVLINK_MSG_ID_REPORT_STATS_MIN_LEN 96
#define MAVLINK_MSG_ID_207_LEN 96
#define MAVLINK_MSG_ID_207_MIN_LEN 96

#define MAVLINK_MSG_ID_REPORT_STATS_CRC 165
#define MAVLINK_MSG_ID_207_CRC 165

#define MAVLINK_MSG_REPORT_STATS_FIELD_DANCE_NAME_LEN 8
#define MAVLINK_MSG_REPORT_STATS_FIELD_DANCE_MD5_LEN 16
#define MAVLINK_MSG_REPORT_STATS_FIELD_PRODUCT_ID_LEN 16

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_REPORT_STATS { \
    207, \
    "REPORT_STATS", \
    28, \
    {  { "utc", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_report_stats_t, utc) }, \
         { "flight_time", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_report_stats_t, flight_time) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_report_stats_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_report_stats_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT16_T, 0, 24, offsetof(mavlink_report_stats_t, alt) }, \
         { "x", NULL, MAVLINK_TYPE_INT16_T, 0, 26, offsetof(mavlink_report_stats_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_INT16_T, 0, 28, offsetof(mavlink_report_stats_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_INT16_T, 0, 30, offsetof(mavlink_report_stats_t, z) }, \
         { "yaw", NULL, MAVLINK_TYPE_UINT16_T, 0, 32, offsetof(mavlink_report_stats_t, yaw) }, \
         { "sensors_present", NULL, MAVLINK_TYPE_UINT16_T, 0, 34, offsetof(mavlink_report_stats_t, sensors_present) }, \
         { "sensors_health", NULL, MAVLINK_TYPE_UINT16_T, 0, 36, offsetof(mavlink_report_stats_t, sensors_health) }, \
         { "drone_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 38, offsetof(mavlink_report_stats_t, drone_id) }, \
         { "firmware_version", NULL, MAVLINK_TYPE_UINT16_T, 0, 40, offsetof(mavlink_report_stats_t, firmware_version) }, \
         { "system_version", NULL, MAVLINK_TYPE_UINT16_T, 0, 42, offsetof(mavlink_report_stats_t, system_version) }, \
         { "dance_version", NULL, MAVLINK_TYPE_UINT16_T, 0, 44, offsetof(mavlink_report_stats_t, dance_version) }, \
         { "dance_name", NULL, MAVLINK_TYPE_UINT8_T, 8, 46, offsetof(mavlink_report_stats_t, dance_name) }, \
         { "dance_md5", NULL, MAVLINK_TYPE_UINT8_T, 16, 54, offsetof(mavlink_report_stats_t, dance_md5) }, \
         { "product_id", NULL, MAVLINK_TYPE_UINT8_T, 16, 70, offsetof(mavlink_report_stats_t, product_id) }, \
         { "aux_token", NULL, MAVLINK_TYPE_UINT8_T, 0, 86, offsetof(mavlink_report_stats_t, aux_token) }, \
         { "time_token", NULL, MAVLINK_TYPE_UINT8_T, 0, 87, offsetof(mavlink_report_stats_t, time_token) }, \
         { "battery_volumn", NULL, MAVLINK_TYPE_UINT8_T, 0, 88, offsetof(mavlink_report_stats_t, battery_volumn) }, \
         { "formation_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 89, offsetof(mavlink_report_stats_t, formation_status) }, \
         { "rgb_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 90, offsetof(mavlink_report_stats_t, rgb_status) }, \
         { "acc_clibration_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 91, offsetof(mavlink_report_stats_t, acc_clibration_status) }, \
         { "mag_clibration_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 92, offsetof(mavlink_report_stats_t, mag_clibration_status) }, \
         { "temperature", NULL, MAVLINK_TYPE_INT8_T, 0, 93, offsetof(mavlink_report_stats_t, temperature) }, \
         { "block_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 94, offsetof(mavlink_report_stats_t, block_status) }, \
         { "drone_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 95, offsetof(mavlink_report_stats_t, drone_status) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_REPORT_STATS { \
    "REPORT_STATS", \
    28, \
    {  { "utc", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_report_stats_t, utc) }, \
         { "flight_time", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_report_stats_t, flight_time) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_report_stats_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_report_stats_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT16_T, 0, 24, offsetof(mavlink_report_stats_t, alt) }, \
         { "x", NULL, MAVLINK_TYPE_INT16_T, 0, 26, offsetof(mavlink_report_stats_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_INT16_T, 0, 28, offsetof(mavlink_report_stats_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_INT16_T, 0, 30, offsetof(mavlink_report_stats_t, z) }, \
         { "yaw", NULL, MAVLINK_TYPE_UINT16_T, 0, 32, offsetof(mavlink_report_stats_t, yaw) }, \
         { "sensors_present", NULL, MAVLINK_TYPE_UINT16_T, 0, 34, offsetof(mavlink_report_stats_t, sensors_present) }, \
         { "sensors_health", NULL, MAVLINK_TYPE_UINT16_T, 0, 36, offsetof(mavlink_report_stats_t, sensors_health) }, \
         { "drone_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 38, offsetof(mavlink_report_stats_t, drone_id) }, \
         { "firmware_version", NULL, MAVLINK_TYPE_UINT16_T, 0, 40, offsetof(mavlink_report_stats_t, firmware_version) }, \
         { "system_version", NULL, MAVLINK_TYPE_UINT16_T, 0, 42, offsetof(mavlink_report_stats_t, system_version) }, \
         { "dance_version", NULL, MAVLINK_TYPE_UINT16_T, 0, 44, offsetof(mavlink_report_stats_t, dance_version) }, \
         { "dance_name", NULL, MAVLINK_TYPE_UINT8_T, 8, 46, offsetof(mavlink_report_stats_t, dance_name) }, \
         { "dance_md5", NULL, MAVLINK_TYPE_UINT8_T, 16, 54, offsetof(mavlink_report_stats_t, dance_md5) }, \
         { "product_id", NULL, MAVLINK_TYPE_UINT8_T, 16, 70, offsetof(mavlink_report_stats_t, product_id) }, \
         { "aux_token", NULL, MAVLINK_TYPE_UINT8_T, 0, 86, offsetof(mavlink_report_stats_t, aux_token) }, \
         { "time_token", NULL, MAVLINK_TYPE_UINT8_T, 0, 87, offsetof(mavlink_report_stats_t, time_token) }, \
         { "battery_volumn", NULL, MAVLINK_TYPE_UINT8_T, 0, 88, offsetof(mavlink_report_stats_t, battery_volumn) }, \
         { "formation_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 89, offsetof(mavlink_report_stats_t, formation_status) }, \
         { "rgb_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 90, offsetof(mavlink_report_stats_t, rgb_status) }, \
         { "acc_clibration_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 91, offsetof(mavlink_report_stats_t, acc_clibration_status) }, \
         { "mag_clibration_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 92, offsetof(mavlink_report_stats_t, mag_clibration_status) }, \
         { "temperature", NULL, MAVLINK_TYPE_INT8_T, 0, 93, offsetof(mavlink_report_stats_t, temperature) }, \
         { "block_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 94, offsetof(mavlink_report_stats_t, block_status) }, \
         { "drone_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 95, offsetof(mavlink_report_stats_t, drone_status) }, \
         } \
}
#endif

/**
 * @brief Pack a report_stats message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param utc  Drone System Time, ms.
 * @param flight_time  Flight time, unit:ms.
 * @param lat  Position X, unit:cm.
 * @param lon  Position Y, unit:cm.
 * @param alt  Position Z, unit:cm.
 * @param x  Formation Position X, unit:cm.
 * @param y  Formation Position Y, unit:cm.
 * @param z  Formation Position Z, unit:cm.
 * @param yaw  Yaw, unit:0.01 degree.
 * @param sensors_present  Sensor Present.
 * @param sensors_health  Sensor Healthy.
 * @param drone_id  Drone ID.
 * @param firmware_version  Version of firmware, 1234 means 1.2.3.4.
 * @param system_version  Version of system, 1234 means 1.2.3.4.
 * @param dance_version  Version of dance, 123 means 1.2.3.
 * @param dance_name  Dance Name.
 * @param dance_md5  Dance Md5.
 * @param product_id  Product ID.
 * @param aux_token  Aux dance step token.
 * @param time_token  Time Synchronization token.
 * @param battery_volumn  Battery volumn.
 * @param formation_status  Formation status.
 * @param rgb_status  RGB status.
 * @param acc_clibration_status  Acc Clibration status.
 * @param mag_clibration_status  Mag Clibration status.
 * @param temperature  Temperature, unit:Centigrade.
 * @param block_status  Block Programming status.
 * @param drone_status  Drone flight status, 0:no fly,1:flying.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_report_stats_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t utc, uint64_t flight_time, int32_t lat, int32_t lon, int16_t alt, int16_t x, int16_t y, int16_t z, uint16_t yaw, uint16_t sensors_present, uint16_t sensors_health, uint16_t drone_id, uint16_t firmware_version, uint16_t system_version, uint16_t dance_version, const uint8_t *dance_name, const uint8_t *dance_md5, const uint8_t *product_id, uint8_t aux_token, uint8_t time_token, uint8_t battery_volumn, uint8_t formation_status, uint8_t rgb_status, uint8_t acc_clibration_status, uint8_t mag_clibration_status, int8_t temperature, uint8_t block_status, uint8_t drone_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_REPORT_STATS_LEN];
    _mav_put_uint64_t(buf, 0, utc);
    _mav_put_uint64_t(buf, 8, flight_time);
    _mav_put_int32_t(buf, 16, lat);
    _mav_put_int32_t(buf, 20, lon);
    _mav_put_int16_t(buf, 24, alt);
    _mav_put_int16_t(buf, 26, x);
    _mav_put_int16_t(buf, 28, y);
    _mav_put_int16_t(buf, 30, z);
    _mav_put_uint16_t(buf, 32, yaw);
    _mav_put_uint16_t(buf, 34, sensors_present);
    _mav_put_uint16_t(buf, 36, sensors_health);
    _mav_put_uint16_t(buf, 38, drone_id);
    _mav_put_uint16_t(buf, 40, firmware_version);
    _mav_put_uint16_t(buf, 42, system_version);
    _mav_put_uint16_t(buf, 44, dance_version);
    _mav_put_uint8_t(buf, 86, aux_token);
    _mav_put_uint8_t(buf, 87, time_token);
    _mav_put_uint8_t(buf, 88, battery_volumn);
    _mav_put_uint8_t(buf, 89, formation_status);
    _mav_put_uint8_t(buf, 90, rgb_status);
    _mav_put_uint8_t(buf, 91, acc_clibration_status);
    _mav_put_uint8_t(buf, 92, mag_clibration_status);
    _mav_put_int8_t(buf, 93, temperature);
    _mav_put_uint8_t(buf, 94, block_status);
    _mav_put_uint8_t(buf, 95, drone_status);
    _mav_put_uint8_t_array(buf, 46, dance_name, 8);
    _mav_put_uint8_t_array(buf, 54, dance_md5, 16);
    _mav_put_uint8_t_array(buf, 70, product_id, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_REPORT_STATS_LEN);
#else
    mavlink_report_stats_t packet;
    packet.utc = utc;
    packet.flight_time = flight_time;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.yaw = yaw;
    packet.sensors_present = sensors_present;
    packet.sensors_health = sensors_health;
    packet.drone_id = drone_id;
    packet.firmware_version = firmware_version;
    packet.system_version = system_version;
    packet.dance_version = dance_version;
    packet.aux_token = aux_token;
    packet.time_token = time_token;
    packet.battery_volumn = battery_volumn;
    packet.formation_status = formation_status;
    packet.rgb_status = rgb_status;
    packet.acc_clibration_status = acc_clibration_status;
    packet.mag_clibration_status = mag_clibration_status;
    packet.temperature = temperature;
    packet.block_status = block_status;
    packet.drone_status = drone_status;
    mav_array_memcpy(packet.dance_name, dance_name, sizeof(uint8_t)*8);
    mav_array_memcpy(packet.dance_md5, dance_md5, sizeof(uint8_t)*16);
    mav_array_memcpy(packet.product_id, product_id, sizeof(uint8_t)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_REPORT_STATS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_REPORT_STATS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_REPORT_STATS_MIN_LEN, MAVLINK_MSG_ID_REPORT_STATS_LEN, MAVLINK_MSG_ID_REPORT_STATS_CRC);
}

/**
 * @brief Pack a report_stats message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param utc  Drone System Time, ms.
 * @param flight_time  Flight time, unit:ms.
 * @param lat  Position X, unit:cm.
 * @param lon  Position Y, unit:cm.
 * @param alt  Position Z, unit:cm.
 * @param x  Formation Position X, unit:cm.
 * @param y  Formation Position Y, unit:cm.
 * @param z  Formation Position Z, unit:cm.
 * @param yaw  Yaw, unit:0.01 degree.
 * @param sensors_present  Sensor Present.
 * @param sensors_health  Sensor Healthy.
 * @param drone_id  Drone ID.
 * @param firmware_version  Version of firmware, 1234 means 1.2.3.4.
 * @param system_version  Version of system, 1234 means 1.2.3.4.
 * @param dance_version  Version of dance, 123 means 1.2.3.
 * @param dance_name  Dance Name.
 * @param dance_md5  Dance Md5.
 * @param product_id  Product ID.
 * @param aux_token  Aux dance step token.
 * @param time_token  Time Synchronization token.
 * @param battery_volumn  Battery volumn.
 * @param formation_status  Formation status.
 * @param rgb_status  RGB status.
 * @param acc_clibration_status  Acc Clibration status.
 * @param mag_clibration_status  Mag Clibration status.
 * @param temperature  Temperature, unit:Centigrade.
 * @param block_status  Block Programming status.
 * @param drone_status  Drone flight status, 0:no fly,1:flying.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_report_stats_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t utc,uint64_t flight_time,int32_t lat,int32_t lon,int16_t alt,int16_t x,int16_t y,int16_t z,uint16_t yaw,uint16_t sensors_present,uint16_t sensors_health,uint16_t drone_id,uint16_t firmware_version,uint16_t system_version,uint16_t dance_version,const uint8_t *dance_name,const uint8_t *dance_md5,const uint8_t *product_id,uint8_t aux_token,uint8_t time_token,uint8_t battery_volumn,uint8_t formation_status,uint8_t rgb_status,uint8_t acc_clibration_status,uint8_t mag_clibration_status,int8_t temperature,uint8_t block_status,uint8_t drone_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_REPORT_STATS_LEN];
    _mav_put_uint64_t(buf, 0, utc);
    _mav_put_uint64_t(buf, 8, flight_time);
    _mav_put_int32_t(buf, 16, lat);
    _mav_put_int32_t(buf, 20, lon);
    _mav_put_int16_t(buf, 24, alt);
    _mav_put_int16_t(buf, 26, x);
    _mav_put_int16_t(buf, 28, y);
    _mav_put_int16_t(buf, 30, z);
    _mav_put_uint16_t(buf, 32, yaw);
    _mav_put_uint16_t(buf, 34, sensors_present);
    _mav_put_uint16_t(buf, 36, sensors_health);
    _mav_put_uint16_t(buf, 38, drone_id);
    _mav_put_uint16_t(buf, 40, firmware_version);
    _mav_put_uint16_t(buf, 42, system_version);
    _mav_put_uint16_t(buf, 44, dance_version);
    _mav_put_uint8_t(buf, 86, aux_token);
    _mav_put_uint8_t(buf, 87, time_token);
    _mav_put_uint8_t(buf, 88, battery_volumn);
    _mav_put_uint8_t(buf, 89, formation_status);
    _mav_put_uint8_t(buf, 90, rgb_status);
    _mav_put_uint8_t(buf, 91, acc_clibration_status);
    _mav_put_uint8_t(buf, 92, mag_clibration_status);
    _mav_put_int8_t(buf, 93, temperature);
    _mav_put_uint8_t(buf, 94, block_status);
    _mav_put_uint8_t(buf, 95, drone_status);
    _mav_put_uint8_t_array(buf, 46, dance_name, 8);
    _mav_put_uint8_t_array(buf, 54, dance_md5, 16);
    _mav_put_uint8_t_array(buf, 70, product_id, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_REPORT_STATS_LEN);
#else
    mavlink_report_stats_t packet;
    packet.utc = utc;
    packet.flight_time = flight_time;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.yaw = yaw;
    packet.sensors_present = sensors_present;
    packet.sensors_health = sensors_health;
    packet.drone_id = drone_id;
    packet.firmware_version = firmware_version;
    packet.system_version = system_version;
    packet.dance_version = dance_version;
    packet.aux_token = aux_token;
    packet.time_token = time_token;
    packet.battery_volumn = battery_volumn;
    packet.formation_status = formation_status;
    packet.rgb_status = rgb_status;
    packet.acc_clibration_status = acc_clibration_status;
    packet.mag_clibration_status = mag_clibration_status;
    packet.temperature = temperature;
    packet.block_status = block_status;
    packet.drone_status = drone_status;
    mav_array_memcpy(packet.dance_name, dance_name, sizeof(uint8_t)*8);
    mav_array_memcpy(packet.dance_md5, dance_md5, sizeof(uint8_t)*16);
    mav_array_memcpy(packet.product_id, product_id, sizeof(uint8_t)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_REPORT_STATS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_REPORT_STATS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_REPORT_STATS_MIN_LEN, MAVLINK_MSG_ID_REPORT_STATS_LEN, MAVLINK_MSG_ID_REPORT_STATS_CRC);
}

/**
 * @brief Encode a report_stats struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param report_stats C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_report_stats_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_report_stats_t* report_stats)
{
    return mavlink_msg_report_stats_pack(system_id, component_id, msg, report_stats->utc, report_stats->flight_time, report_stats->lat, report_stats->lon, report_stats->alt, report_stats->x, report_stats->y, report_stats->z, report_stats->yaw, report_stats->sensors_present, report_stats->sensors_health, report_stats->drone_id, report_stats->firmware_version, report_stats->system_version, report_stats->dance_version, report_stats->dance_name, report_stats->dance_md5, report_stats->product_id, report_stats->aux_token, report_stats->time_token, report_stats->battery_volumn, report_stats->formation_status, report_stats->rgb_status, report_stats->acc_clibration_status, report_stats->mag_clibration_status, report_stats->temperature, report_stats->block_status, report_stats->drone_status);
}

/**
 * @brief Encode a report_stats struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param report_stats C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_report_stats_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_report_stats_t* report_stats)
{
    return mavlink_msg_report_stats_pack_chan(system_id, component_id, chan, msg, report_stats->utc, report_stats->flight_time, report_stats->lat, report_stats->lon, report_stats->alt, report_stats->x, report_stats->y, report_stats->z, report_stats->yaw, report_stats->sensors_present, report_stats->sensors_health, report_stats->drone_id, report_stats->firmware_version, report_stats->system_version, report_stats->dance_version, report_stats->dance_name, report_stats->dance_md5, report_stats->product_id, report_stats->aux_token, report_stats->time_token, report_stats->battery_volumn, report_stats->formation_status, report_stats->rgb_status, report_stats->acc_clibration_status, report_stats->mag_clibration_status, report_stats->temperature, report_stats->block_status, report_stats->drone_status);
}

/**
 * @brief Send a report_stats message
 * @param chan MAVLink channel to send the message
 *
 * @param utc  Drone System Time, ms.
 * @param flight_time  Flight time, unit:ms.
 * @param lat  Position X, unit:cm.
 * @param lon  Position Y, unit:cm.
 * @param alt  Position Z, unit:cm.
 * @param x  Formation Position X, unit:cm.
 * @param y  Formation Position Y, unit:cm.
 * @param z  Formation Position Z, unit:cm.
 * @param yaw  Yaw, unit:0.01 degree.
 * @param sensors_present  Sensor Present.
 * @param sensors_health  Sensor Healthy.
 * @param drone_id  Drone ID.
 * @param firmware_version  Version of firmware, 1234 means 1.2.3.4.
 * @param system_version  Version of system, 1234 means 1.2.3.4.
 * @param dance_version  Version of dance, 123 means 1.2.3.
 * @param dance_name  Dance Name.
 * @param dance_md5  Dance Md5.
 * @param product_id  Product ID.
 * @param aux_token  Aux dance step token.
 * @param time_token  Time Synchronization token.
 * @param battery_volumn  Battery volumn.
 * @param formation_status  Formation status.
 * @param rgb_status  RGB status.
 * @param acc_clibration_status  Acc Clibration status.
 * @param mag_clibration_status  Mag Clibration status.
 * @param temperature  Temperature, unit:Centigrade.
 * @param block_status  Block Programming status.
 * @param drone_status  Drone flight status, 0:no fly,1:flying.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_report_stats_send(mavlink_channel_t chan, uint64_t utc, uint64_t flight_time, int32_t lat, int32_t lon, int16_t alt, int16_t x, int16_t y, int16_t z, uint16_t yaw, uint16_t sensors_present, uint16_t sensors_health, uint16_t drone_id, uint16_t firmware_version, uint16_t system_version, uint16_t dance_version, const uint8_t *dance_name, const uint8_t *dance_md5, const uint8_t *product_id, uint8_t aux_token, uint8_t time_token, uint8_t battery_volumn, uint8_t formation_status, uint8_t rgb_status, uint8_t acc_clibration_status, uint8_t mag_clibration_status, int8_t temperature, uint8_t block_status, uint8_t drone_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_REPORT_STATS_LEN];
    _mav_put_uint64_t(buf, 0, utc);
    _mav_put_uint64_t(buf, 8, flight_time);
    _mav_put_int32_t(buf, 16, lat);
    _mav_put_int32_t(buf, 20, lon);
    _mav_put_int16_t(buf, 24, alt);
    _mav_put_int16_t(buf, 26, x);
    _mav_put_int16_t(buf, 28, y);
    _mav_put_int16_t(buf, 30, z);
    _mav_put_uint16_t(buf, 32, yaw);
    _mav_put_uint16_t(buf, 34, sensors_present);
    _mav_put_uint16_t(buf, 36, sensors_health);
    _mav_put_uint16_t(buf, 38, drone_id);
    _mav_put_uint16_t(buf, 40, firmware_version);
    _mav_put_uint16_t(buf, 42, system_version);
    _mav_put_uint16_t(buf, 44, dance_version);
    _mav_put_uint8_t(buf, 86, aux_token);
    _mav_put_uint8_t(buf, 87, time_token);
    _mav_put_uint8_t(buf, 88, battery_volumn);
    _mav_put_uint8_t(buf, 89, formation_status);
    _mav_put_uint8_t(buf, 90, rgb_status);
    _mav_put_uint8_t(buf, 91, acc_clibration_status);
    _mav_put_uint8_t(buf, 92, mag_clibration_status);
    _mav_put_int8_t(buf, 93, temperature);
    _mav_put_uint8_t(buf, 94, block_status);
    _mav_put_uint8_t(buf, 95, drone_status);
    _mav_put_uint8_t_array(buf, 46, dance_name, 8);
    _mav_put_uint8_t_array(buf, 54, dance_md5, 16);
    _mav_put_uint8_t_array(buf, 70, product_id, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REPORT_STATS, buf, MAVLINK_MSG_ID_REPORT_STATS_MIN_LEN, MAVLINK_MSG_ID_REPORT_STATS_LEN, MAVLINK_MSG_ID_REPORT_STATS_CRC);
#else
    mavlink_report_stats_t packet;
    packet.utc = utc;
    packet.flight_time = flight_time;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.yaw = yaw;
    packet.sensors_present = sensors_present;
    packet.sensors_health = sensors_health;
    packet.drone_id = drone_id;
    packet.firmware_version = firmware_version;
    packet.system_version = system_version;
    packet.dance_version = dance_version;
    packet.aux_token = aux_token;
    packet.time_token = time_token;
    packet.battery_volumn = battery_volumn;
    packet.formation_status = formation_status;
    packet.rgb_status = rgb_status;
    packet.acc_clibration_status = acc_clibration_status;
    packet.mag_clibration_status = mag_clibration_status;
    packet.temperature = temperature;
    packet.block_status = block_status;
    packet.drone_status = drone_status;
    mav_array_memcpy(packet.dance_name, dance_name, sizeof(uint8_t)*8);
    mav_array_memcpy(packet.dance_md5, dance_md5, sizeof(uint8_t)*16);
    mav_array_memcpy(packet.product_id, product_id, sizeof(uint8_t)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REPORT_STATS, (const char *)&packet, MAVLINK_MSG_ID_REPORT_STATS_MIN_LEN, MAVLINK_MSG_ID_REPORT_STATS_LEN, MAVLINK_MSG_ID_REPORT_STATS_CRC);
#endif
}

/**
 * @brief Send a report_stats message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_report_stats_send_struct(mavlink_channel_t chan, const mavlink_report_stats_t* report_stats)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_report_stats_send(chan, report_stats->utc, report_stats->flight_time, report_stats->lat, report_stats->lon, report_stats->alt, report_stats->x, report_stats->y, report_stats->z, report_stats->yaw, report_stats->sensors_present, report_stats->sensors_health, report_stats->drone_id, report_stats->firmware_version, report_stats->system_version, report_stats->dance_version, report_stats->dance_name, report_stats->dance_md5, report_stats->product_id, report_stats->aux_token, report_stats->time_token, report_stats->battery_volumn, report_stats->formation_status, report_stats->rgb_status, report_stats->acc_clibration_status, report_stats->mag_clibration_status, report_stats->temperature, report_stats->block_status, report_stats->drone_status);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REPORT_STATS, (const char *)report_stats, MAVLINK_MSG_ID_REPORT_STATS_MIN_LEN, MAVLINK_MSG_ID_REPORT_STATS_LEN, MAVLINK_MSG_ID_REPORT_STATS_CRC);
#endif
}

#if MAVLINK_MSG_ID_REPORT_STATS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_report_stats_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t utc, uint64_t flight_time, int32_t lat, int32_t lon, int16_t alt, int16_t x, int16_t y, int16_t z, uint16_t yaw, uint16_t sensors_present, uint16_t sensors_health, uint16_t drone_id, uint16_t firmware_version, uint16_t system_version, uint16_t dance_version, const uint8_t *dance_name, const uint8_t *dance_md5, const uint8_t *product_id, uint8_t aux_token, uint8_t time_token, uint8_t battery_volumn, uint8_t formation_status, uint8_t rgb_status, uint8_t acc_clibration_status, uint8_t mag_clibration_status, int8_t temperature, uint8_t block_status, uint8_t drone_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, utc);
    _mav_put_uint64_t(buf, 8, flight_time);
    _mav_put_int32_t(buf, 16, lat);
    _mav_put_int32_t(buf, 20, lon);
    _mav_put_int16_t(buf, 24, alt);
    _mav_put_int16_t(buf, 26, x);
    _mav_put_int16_t(buf, 28, y);
    _mav_put_int16_t(buf, 30, z);
    _mav_put_uint16_t(buf, 32, yaw);
    _mav_put_uint16_t(buf, 34, sensors_present);
    _mav_put_uint16_t(buf, 36, sensors_health);
    _mav_put_uint16_t(buf, 38, drone_id);
    _mav_put_uint16_t(buf, 40, firmware_version);
    _mav_put_uint16_t(buf, 42, system_version);
    _mav_put_uint16_t(buf, 44, dance_version);
    _mav_put_uint8_t(buf, 86, aux_token);
    _mav_put_uint8_t(buf, 87, time_token);
    _mav_put_uint8_t(buf, 88, battery_volumn);
    _mav_put_uint8_t(buf, 89, formation_status);
    _mav_put_uint8_t(buf, 90, rgb_status);
    _mav_put_uint8_t(buf, 91, acc_clibration_status);
    _mav_put_uint8_t(buf, 92, mag_clibration_status);
    _mav_put_int8_t(buf, 93, temperature);
    _mav_put_uint8_t(buf, 94, block_status);
    _mav_put_uint8_t(buf, 95, drone_status);
    _mav_put_uint8_t_array(buf, 46, dance_name, 8);
    _mav_put_uint8_t_array(buf, 54, dance_md5, 16);
    _mav_put_uint8_t_array(buf, 70, product_id, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REPORT_STATS, buf, MAVLINK_MSG_ID_REPORT_STATS_MIN_LEN, MAVLINK_MSG_ID_REPORT_STATS_LEN, MAVLINK_MSG_ID_REPORT_STATS_CRC);
#else
    mavlink_report_stats_t *packet = (mavlink_report_stats_t *)msgbuf;
    packet->utc = utc;
    packet->flight_time = flight_time;
    packet->lat = lat;
    packet->lon = lon;
    packet->alt = alt;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->yaw = yaw;
    packet->sensors_present = sensors_present;
    packet->sensors_health = sensors_health;
    packet->drone_id = drone_id;
    packet->firmware_version = firmware_version;
    packet->system_version = system_version;
    packet->dance_version = dance_version;
    packet->aux_token = aux_token;
    packet->time_token = time_token;
    packet->battery_volumn = battery_volumn;
    packet->formation_status = formation_status;
    packet->rgb_status = rgb_status;
    packet->acc_clibration_status = acc_clibration_status;
    packet->mag_clibration_status = mag_clibration_status;
    packet->temperature = temperature;
    packet->block_status = block_status;
    packet->drone_status = drone_status;
    mav_array_memcpy(packet->dance_name, dance_name, sizeof(uint8_t)*8);
    mav_array_memcpy(packet->dance_md5, dance_md5, sizeof(uint8_t)*16);
    mav_array_memcpy(packet->product_id, product_id, sizeof(uint8_t)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REPORT_STATS, (const char *)packet, MAVLINK_MSG_ID_REPORT_STATS_MIN_LEN, MAVLINK_MSG_ID_REPORT_STATS_LEN, MAVLINK_MSG_ID_REPORT_STATS_CRC);
#endif
}
#endif

#endif

// MESSAGE REPORT_STATS UNPACKING


/**
 * @brief Get field utc from report_stats message
 *
 * @return  Drone System Time, ms.
 */
static inline uint64_t mavlink_msg_report_stats_get_utc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field flight_time from report_stats message
 *
 * @return  Flight time, unit:ms.
 */
static inline uint64_t mavlink_msg_report_stats_get_flight_time(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Get field lat from report_stats message
 *
 * @return  Position X, unit:cm.
 */
static inline int32_t mavlink_msg_report_stats_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field lon from report_stats message
 *
 * @return  Position Y, unit:cm.
 */
static inline int32_t mavlink_msg_report_stats_get_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  20);
}

/**
 * @brief Get field alt from report_stats message
 *
 * @return  Position Z, unit:cm.
 */
static inline int16_t mavlink_msg_report_stats_get_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  24);
}

/**
 * @brief Get field x from report_stats message
 *
 * @return  Formation Position X, unit:cm.
 */
static inline int16_t mavlink_msg_report_stats_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  26);
}

/**
 * @brief Get field y from report_stats message
 *
 * @return  Formation Position Y, unit:cm.
 */
static inline int16_t mavlink_msg_report_stats_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  28);
}

/**
 * @brief Get field z from report_stats message
 *
 * @return  Formation Position Z, unit:cm.
 */
static inline int16_t mavlink_msg_report_stats_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  30);
}

/**
 * @brief Get field yaw from report_stats message
 *
 * @return  Yaw, unit:0.01 degree.
 */
static inline uint16_t mavlink_msg_report_stats_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  32);
}

/**
 * @brief Get field sensors_present from report_stats message
 *
 * @return  Sensor Present.
 */
static inline uint16_t mavlink_msg_report_stats_get_sensors_present(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  34);
}

/**
 * @brief Get field sensors_health from report_stats message
 *
 * @return  Sensor Healthy.
 */
static inline uint16_t mavlink_msg_report_stats_get_sensors_health(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  36);
}

/**
 * @brief Get field drone_id from report_stats message
 *
 * @return  Drone ID.
 */
static inline uint16_t mavlink_msg_report_stats_get_drone_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  38);
}

/**
 * @brief Get field firmware_version from report_stats message
 *
 * @return  Version of firmware, 1234 means 1.2.3.4.
 */
static inline uint16_t mavlink_msg_report_stats_get_firmware_version(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  40);
}

/**
 * @brief Get field system_version from report_stats message
 *
 * @return  Version of system, 1234 means 1.2.3.4.
 */
static inline uint16_t mavlink_msg_report_stats_get_system_version(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  42);
}

/**
 * @brief Get field dance_version from report_stats message
 *
 * @return  Version of dance, 123 means 1.2.3.
 */
static inline uint16_t mavlink_msg_report_stats_get_dance_version(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  44);
}

/**
 * @brief Get field dance_name from report_stats message
 *
 * @return  Dance Name.
 */
static inline uint16_t mavlink_msg_report_stats_get_dance_name(const mavlink_message_t* msg, uint8_t *dance_name)
{
    return _MAV_RETURN_uint8_t_array(msg, dance_name, 8,  46);
}

/**
 * @brief Get field dance_md5 from report_stats message
 *
 * @return  Dance Md5.
 */
static inline uint16_t mavlink_msg_report_stats_get_dance_md5(const mavlink_message_t* msg, uint8_t *dance_md5)
{
    return _MAV_RETURN_uint8_t_array(msg, dance_md5, 16,  54);
}

/**
 * @brief Get field product_id from report_stats message
 *
 * @return  Product ID.
 */
static inline uint16_t mavlink_msg_report_stats_get_product_id(const mavlink_message_t* msg, uint8_t *product_id)
{
    return _MAV_RETURN_uint8_t_array(msg, product_id, 16,  70);
}

/**
 * @brief Get field aux_token from report_stats message
 *
 * @return  Aux dance step token.
 */
static inline uint8_t mavlink_msg_report_stats_get_aux_token(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  86);
}

/**
 * @brief Get field time_token from report_stats message
 *
 * @return  Time Synchronization token.
 */
static inline uint8_t mavlink_msg_report_stats_get_time_token(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  87);
}

/**
 * @brief Get field battery_volumn from report_stats message
 *
 * @return  Battery volumn.
 */
static inline uint8_t mavlink_msg_report_stats_get_battery_volumn(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  88);
}

/**
 * @brief Get field formation_status from report_stats message
 *
 * @return  Formation status.
 */
static inline uint8_t mavlink_msg_report_stats_get_formation_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  89);
}

/**
 * @brief Get field rgb_status from report_stats message
 *
 * @return  RGB status.
 */
static inline uint8_t mavlink_msg_report_stats_get_rgb_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  90);
}

/**
 * @brief Get field acc_clibration_status from report_stats message
 *
 * @return  Acc Clibration status.
 */
static inline uint8_t mavlink_msg_report_stats_get_acc_clibration_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  91);
}

/**
 * @brief Get field mag_clibration_status from report_stats message
 *
 * @return  Mag Clibration status.
 */
static inline uint8_t mavlink_msg_report_stats_get_mag_clibration_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  92);
}

/**
 * @brief Get field temperature from report_stats message
 *
 * @return  Temperature, unit:Centigrade.
 */
static inline int8_t mavlink_msg_report_stats_get_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  93);
}

/**
 * @brief Get field block_status from report_stats message
 *
 * @return  Block Programming status.
 */
static inline uint8_t mavlink_msg_report_stats_get_block_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  94);
}

/**
 * @brief Get field drone_status from report_stats message
 *
 * @return  Drone flight status, 0:no fly,1:flying.
 */
static inline uint8_t mavlink_msg_report_stats_get_drone_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  95);
}

/**
 * @brief Decode a report_stats message into a struct
 *
 * @param msg The message to decode
 * @param report_stats C-struct to decode the message contents into
 */
static inline void mavlink_msg_report_stats_decode(const mavlink_message_t* msg, mavlink_report_stats_t* report_stats)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    report_stats->utc = mavlink_msg_report_stats_get_utc(msg);
    report_stats->flight_time = mavlink_msg_report_stats_get_flight_time(msg);
    report_stats->lat = mavlink_msg_report_stats_get_lat(msg);
    report_stats->lon = mavlink_msg_report_stats_get_lon(msg);
    report_stats->alt = mavlink_msg_report_stats_get_alt(msg);
    report_stats->x = mavlink_msg_report_stats_get_x(msg);
    report_stats->y = mavlink_msg_report_stats_get_y(msg);
    report_stats->z = mavlink_msg_report_stats_get_z(msg);
    report_stats->yaw = mavlink_msg_report_stats_get_yaw(msg);
    report_stats->sensors_present = mavlink_msg_report_stats_get_sensors_present(msg);
    report_stats->sensors_health = mavlink_msg_report_stats_get_sensors_health(msg);
    report_stats->drone_id = mavlink_msg_report_stats_get_drone_id(msg);
    report_stats->firmware_version = mavlink_msg_report_stats_get_firmware_version(msg);
    report_stats->system_version = mavlink_msg_report_stats_get_system_version(msg);
    report_stats->dance_version = mavlink_msg_report_stats_get_dance_version(msg);
    mavlink_msg_report_stats_get_dance_name(msg, report_stats->dance_name);
    mavlink_msg_report_stats_get_dance_md5(msg, report_stats->dance_md5);
    mavlink_msg_report_stats_get_product_id(msg, report_stats->product_id);
    report_stats->aux_token = mavlink_msg_report_stats_get_aux_token(msg);
    report_stats->time_token = mavlink_msg_report_stats_get_time_token(msg);
    report_stats->battery_volumn = mavlink_msg_report_stats_get_battery_volumn(msg);
    report_stats->formation_status = mavlink_msg_report_stats_get_formation_status(msg);
    report_stats->rgb_status = mavlink_msg_report_stats_get_rgb_status(msg);
    report_stats->acc_clibration_status = mavlink_msg_report_stats_get_acc_clibration_status(msg);
    report_stats->mag_clibration_status = mavlink_msg_report_stats_get_mag_clibration_status(msg);
    report_stats->temperature = mavlink_msg_report_stats_get_temperature(msg);
    report_stats->block_status = mavlink_msg_report_stats_get_block_status(msg);
    report_stats->drone_status = mavlink_msg_report_stats_get_drone_status(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_REPORT_STATS_LEN? msg->len : MAVLINK_MSG_ID_REPORT_STATS_LEN;
        memset(report_stats, 0, MAVLINK_MSG_ID_REPORT_STATS_LEN);
    memcpy(report_stats, _MAV_PAYLOAD(msg), len);
#endif
}