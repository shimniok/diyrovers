// MESSAGE GPS_RAW_UGV PACKING

#define MAVLINK_MSG_ID_GPS_RAW_UGV 86

typedef struct __mavlink_gps_raw_ugv_t 
{
    uint64_t usec; ///< Timestamp (microseconds since UNIX epoch or microseconds since system boot)
    uint8_t fix_type; ///< 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
    double lat; ///< Latitude in 1E15 degrees
    double lon; ///< Longitude in 1E15 degrees
    float eph; ///< GPS HDOP
    float epv; ///< GPS VDOP
    float v; ///< GPS ground speed
    float hdg; ///< Compass heading in degrees, 0..360 degrees

} mavlink_gps_raw_ugv_t;


/**
 * @brief Pack a gps_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param lat Latitude in degrees
 * @param lon Longitude in degrees
 * @param eph GPS HDOP
 * @param epv GPS VDOP
 * @param v GPS ground speed
 * @param hdg Compass heading in degrees, 0..360 degrees
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_raw_ugv_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint64_t usec, uint8_t fix_type, double lat, double lon, float eph, float epv, float v, float hdg)
{
    uint16_t i = 0;
    msg->msgid = MAVLINK_MSG_ID_GPS_RAW_UGV;

    i += put_uint64_t_by_index(usec, i, msg->payload); // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
    i += put_uint8_t_by_index(fix_type, i, msg->payload); // 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
    i += put_double_by_index(lat, i, msg->payload); // Latitude in degrees
    i += put_double_by_index(lon, i, msg->payload); // Longitude in degrees
    i += put_float_by_index(eph, i, msg->payload); // GPS HDOP
    i += put_float_by_index(epv, i, msg->payload); // GPS VDOP
    i += put_float_by_index(v, i, msg->payload); // GPS ground speed
    i += put_float_by_index(hdg, i, msg->payload); // Compass heading in degrees, 0..360 degrees

    return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a gps_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param lat Latitude in degrees
 * @param lon Longitude in degrees
 * @param eph GPS HDOP
 * @param epv GPS VDOP
 * @param v GPS ground speed
 * @param hdg Compass heading in degrees, 0..360 degrees
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_raw_ugv_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint64_t usec, uint8_t fix_type, double lat, double lon, float eph, float epv, float v, float hdg)
{
    uint16_t i = 0;
    msg->msgid = MAVLINK_MSG_ID_GPS_RAW_UGV;

    i += put_uint64_t_by_index(usec, i, msg->payload); // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
    i += put_uint8_t_by_index(fix_type, i, msg->payload); // 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
    i += put_double_by_index(lat, i, msg->payload); // Latitude in degrees
    i += put_double_by_index(lon, i, msg->payload); // Longitude in degrees
    i += put_float_by_index(eph, i, msg->payload); // GPS HDOP
    i += put_float_by_index(epv, i, msg->payload); // GPS VDOP
    i += put_float_by_index(v, i, msg->payload); // GPS ground speed
    i += put_float_by_index(hdg, i, msg->payload); // Compass heading in degrees, 0..360 degrees

    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a gps_raw struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gps_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gps_raw_ugv_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gps_raw_ugv_t* gps_raw)
{
    return mavlink_msg_gps_raw_ugv_pack(system_id, component_id, msg, gps_raw->usec, gps_raw->fix_type, gps_raw->lat, gps_raw->lon, gps_raw->eph, gps_raw->epv, gps_raw->v, gps_raw->hdg);
}

/**
 * @brief Send a gps_raw message
 * @param chan MAVLink channel to send the message
 *
 * @param usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param lat Latitude in degrees
 * @param lon Longitude in degrees
 * @param eph GPS HDOP
 * @param epv GPS VDOP
 * @param v GPS ground speed
 * @param hdg Compass heading in degrees, 0..360 degrees
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gps_raw_ugv_send(mavlink_channel_t chan, uint64_t usec, uint8_t fix_type, double lat, double lon, float eph, float epv, float v, float hdg)
{
    mavlink_message_t msg;
    mavlink_msg_gps_raw_ugv_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, usec, fix_type, lat, lon, eph, epv, v, hdg);
    mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE GPS_RAW UNPACKING

/**
 * @brief Get field usec from gps_raw message
 *
 * @return Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 */
static inline uint64_t mavlink_msg_gps_raw_ugv_get_usec(const mavlink_message_t* msg)
{
    generic_64bit r;
    r.b[7] = (msg->payload)[0];
    r.b[6] = (msg->payload)[1];
    r.b[5] = (msg->payload)[2];
    r.b[4] = (msg->payload)[3];
    r.b[3] = (msg->payload)[4];
    r.b[2] = (msg->payload)[5];
    r.b[1] = (msg->payload)[6];
    r.b[0] = (msg->payload)[7];
    return (uint64_t)r.ll;
}

/**
 * @brief Get field fix_type from gps_raw message
 *
 * @return 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 */
static inline uint8_t mavlink_msg_gps_raw_ugv_get_fix_type(const mavlink_message_t* msg)
{
    return (uint8_t)(msg->payload+sizeof(uint64_t))[0];
}

/**
 * @brief Get field lat from gps_raw message
 *
 * @return Latitude in degrees
 */
static inline double mavlink_msg_gps_raw_ugv_get_lat(const mavlink_message_t* msg)
{
    generic_64bit r;
    r.b[7] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t))[0];
    r.b[6] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t))[1];
    r.b[5] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t))[2];
    r.b[4] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t))[3];
    r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t))[4];
    r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t))[5];
    r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t))[6];
    r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t))[7];
    return (double)r.d;
}

/**
 * @brief Get field lon from gps_raw message
 *
 * @return Longitude in degrees
 */
static inline double mavlink_msg_gps_raw_ugv_get_lon(const mavlink_message_t* msg)
{
    generic_64bit r;
    r.b[7] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(double))[0];
    r.b[6] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(double))[1];
    r.b[5] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(double))[2];
    r.b[4] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(double))[3];
    r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(double))[4];
    r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(double))[5];
    r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(double))[6];
    r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(double))[7];
    return (double)r.d;
}

/**
 * @brief Get field eph from gps_raw message
 *
 * @return GPS HDOP
 */
static inline float mavlink_msg_gps_raw_ugv_get_eph(const mavlink_message_t* msg)
{
    generic_32bit r;
    r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(double)+sizeof(double)+sizeof(float))[0];
    r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(double)+sizeof(double)+sizeof(float))[1];
    r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(double)+sizeof(double)+sizeof(float))[2];
    r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(double)+sizeof(double)+sizeof(float))[3];
    return (float)r.f;
}

/**
 * @brief Get field epv from gps_raw message
 *
 * @return GPS VDOP
 */
static inline float mavlink_msg_gps_raw_ugv_get_epv(const mavlink_message_t* msg)
{
    generic_32bit r;
    r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(double)+sizeof(double)+sizeof(float)+sizeof(float))[0];
    r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(double)+sizeof(double)+sizeof(float)+sizeof(float))[1];
    r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(double)+sizeof(double)+sizeof(float)+sizeof(float))[2];
    r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(double)+sizeof(double)+sizeof(float)+sizeof(float))[3];
    return (float)r.f;
}

/**
 * @brief Get field v from gps_raw message
 *
 * @return GPS ground speed
 */
static inline float mavlink_msg_gps_raw_ugv_get_v(const mavlink_message_t* msg)
{
    generic_32bit r;
    r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(double)+sizeof(double)+sizeof(float)+sizeof(float)+sizeof(float))[0];
    r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(double)+sizeof(double)+sizeof(float)+sizeof(float)+sizeof(float))[1];
    r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(double)+sizeof(double)+sizeof(float)+sizeof(float)+sizeof(float))[2];
    r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(double)+sizeof(double)+sizeof(float)+sizeof(float)+sizeof(float))[3];
    return (float)r.f;
}

/**
 * @brief Get field hdg from gps_raw message
 *
 * @return Compass heading in degrees, 0..360 degrees
 */
static inline float mavlink_msg_gps_raw_ugv_get_hdg(const mavlink_message_t* msg)
{
    generic_32bit r;
    r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(double)+sizeof(double)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
    r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(double)+sizeof(double)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
    r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(double)+sizeof(double)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
    r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(double)+sizeof(double)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
    return (float)r.f;
}

/**
 * @brief Decode a gps_raw message into a struct
 *
 * @param msg The message to decode
 * @param gps_raw C-struct to decode the message contents into
 */
static inline void mavlink_msg_gps_raw_ugv_decode(const mavlink_message_t* msg, mavlink_gps_raw_ugv_t* gps_raw)
{
    gps_raw->usec = mavlink_msg_gps_raw_ugv_get_usec(msg);
    gps_raw->fix_type = mavlink_msg_gps_raw_ugv_get_fix_type(msg);
    gps_raw->lat = mavlink_msg_gps_raw_ugv_get_lat(msg);
    gps_raw->lon = mavlink_msg_gps_raw_ugv_get_lon(msg);
    gps_raw->eph = mavlink_msg_gps_raw_ugv_get_eph(msg);
    gps_raw->epv = mavlink_msg_gps_raw_ugv_get_epv(msg);
    gps_raw->v = mavlink_msg_gps_raw_ugv_get_v(msg);
    gps_raw->hdg = mavlink_msg_gps_raw_ugv_get_hdg(msg);
}
