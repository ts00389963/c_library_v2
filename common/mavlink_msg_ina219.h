#pragma once
// MESSAGE INA219 PACKING

#define MAVLINK_MSG_ID_INA219 400

MAVPACKED(
typedef struct __mavlink_ina219_t {
 uint64_t timestamp; /*<  Time since system start*/
 float voltageIn; /*< [V] Voltage in volts*/
 float currentIn; /*< [A] Current in amperes*/
 float powerIn; /*< [W] Power in watts*/
 float voltageOut; /*< [V] Voltage in volts*/
 float currentOut; /*< [A] Current in amperes*/
 float powerOut; /*< [W] Power in watts*/
}) mavlink_ina219_t;

#define MAVLINK_MSG_ID_INA219_LEN 32
#define MAVLINK_MSG_ID_INA219_MIN_LEN 32
#define MAVLINK_MSG_ID_400_LEN 32
#define MAVLINK_MSG_ID_400_MIN_LEN 32

#define MAVLINK_MSG_ID_INA219_CRC 201
#define MAVLINK_MSG_ID_400_CRC 201



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_INA219 { \
    400, \
    "INA219", \
    7, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_ina219_t, timestamp) }, \
         { "voltageIn", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_ina219_t, voltageIn) }, \
         { "currentIn", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_ina219_t, currentIn) }, \
         { "powerIn", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_ina219_t, powerIn) }, \
         { "voltageOut", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_ina219_t, voltageOut) }, \
         { "currentOut", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_ina219_t, currentOut) }, \
         { "powerOut", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_ina219_t, powerOut) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_INA219 { \
    "INA219", \
    7, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_ina219_t, timestamp) }, \
         { "voltageIn", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_ina219_t, voltageIn) }, \
         { "currentIn", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_ina219_t, currentIn) }, \
         { "powerIn", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_ina219_t, powerIn) }, \
         { "voltageOut", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_ina219_t, voltageOut) }, \
         { "currentOut", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_ina219_t, currentOut) }, \
         { "powerOut", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_ina219_t, powerOut) }, \
         } \
}
#endif

/**
 * @brief Pack a ina219 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp  Time since system start
 * @param voltageIn [V] Voltage in volts
 * @param currentIn [A] Current in amperes
 * @param powerIn [W] Power in watts
 * @param voltageOut [V] Voltage in volts
 * @param currentOut [A] Current in amperes
 * @param powerOut [W] Power in watts
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ina219_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, float voltageIn, float currentIn, float powerIn, float voltageOut, float currentOut, float powerOut)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_INA219_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, voltageIn);
    _mav_put_float(buf, 12, currentIn);
    _mav_put_float(buf, 16, powerIn);
    _mav_put_float(buf, 20, voltageOut);
    _mav_put_float(buf, 24, currentOut);
    _mav_put_float(buf, 28, powerOut);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_INA219_LEN);
#else
    mavlink_ina219_t packet;
    packet.timestamp = timestamp;
    packet.voltageIn = voltageIn;
    packet.currentIn = currentIn;
    packet.powerIn = powerIn;
    packet.voltageOut = voltageOut;
    packet.currentOut = currentOut;
    packet.powerOut = powerOut;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_INA219_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_INA219;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_INA219_MIN_LEN, MAVLINK_MSG_ID_INA219_LEN, MAVLINK_MSG_ID_INA219_CRC);
}

/**
 * @brief Pack a ina219 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp  Time since system start
 * @param voltageIn [V] Voltage in volts
 * @param currentIn [A] Current in amperes
 * @param powerIn [W] Power in watts
 * @param voltageOut [V] Voltage in volts
 * @param currentOut [A] Current in amperes
 * @param powerOut [W] Power in watts
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ina219_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,float voltageIn,float currentIn,float powerIn,float voltageOut,float currentOut,float powerOut)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_INA219_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, voltageIn);
    _mav_put_float(buf, 12, currentIn);
    _mav_put_float(buf, 16, powerIn);
    _mav_put_float(buf, 20, voltageOut);
    _mav_put_float(buf, 24, currentOut);
    _mav_put_float(buf, 28, powerOut);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_INA219_LEN);
#else
    mavlink_ina219_t packet;
    packet.timestamp = timestamp;
    packet.voltageIn = voltageIn;
    packet.currentIn = currentIn;
    packet.powerIn = powerIn;
    packet.voltageOut = voltageOut;
    packet.currentOut = currentOut;
    packet.powerOut = powerOut;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_INA219_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_INA219;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_INA219_MIN_LEN, MAVLINK_MSG_ID_INA219_LEN, MAVLINK_MSG_ID_INA219_CRC);
}

/**
 * @brief Encode a ina219 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ina219 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ina219_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ina219_t* ina219)
{
    return mavlink_msg_ina219_pack(system_id, component_id, msg, ina219->timestamp, ina219->voltageIn, ina219->currentIn, ina219->powerIn, ina219->voltageOut, ina219->currentOut, ina219->powerOut);
}

/**
 * @brief Encode a ina219 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ina219 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ina219_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ina219_t* ina219)
{
    return mavlink_msg_ina219_pack_chan(system_id, component_id, chan, msg, ina219->timestamp, ina219->voltageIn, ina219->currentIn, ina219->powerIn, ina219->voltageOut, ina219->currentOut, ina219->powerOut);
}

/**
 * @brief Send a ina219 message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp  Time since system start
 * @param voltageIn [V] Voltage in volts
 * @param currentIn [A] Current in amperes
 * @param powerIn [W] Power in watts
 * @param voltageOut [V] Voltage in volts
 * @param currentOut [A] Current in amperes
 * @param powerOut [W] Power in watts
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ina219_send(mavlink_channel_t chan, uint64_t timestamp, float voltageIn, float currentIn, float powerIn, float voltageOut, float currentOut, float powerOut)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_INA219_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, voltageIn);
    _mav_put_float(buf, 12, currentIn);
    _mav_put_float(buf, 16, powerIn);
    _mav_put_float(buf, 20, voltageOut);
    _mav_put_float(buf, 24, currentOut);
    _mav_put_float(buf, 28, powerOut);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INA219, buf, MAVLINK_MSG_ID_INA219_MIN_LEN, MAVLINK_MSG_ID_INA219_LEN, MAVLINK_MSG_ID_INA219_CRC);
#else
    mavlink_ina219_t packet;
    packet.timestamp = timestamp;
    packet.voltageIn = voltageIn;
    packet.currentIn = currentIn;
    packet.powerIn = powerIn;
    packet.voltageOut = voltageOut;
    packet.currentOut = currentOut;
    packet.powerOut = powerOut;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INA219, (const char *)&packet, MAVLINK_MSG_ID_INA219_MIN_LEN, MAVLINK_MSG_ID_INA219_LEN, MAVLINK_MSG_ID_INA219_CRC);
#endif
}

/**
 * @brief Send a ina219 message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_ina219_send_struct(mavlink_channel_t chan, const mavlink_ina219_t* ina219)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_ina219_send(chan, ina219->timestamp, ina219->voltageIn, ina219->currentIn, ina219->powerIn, ina219->voltageOut, ina219->currentOut, ina219->powerOut);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INA219, (const char *)ina219, MAVLINK_MSG_ID_INA219_MIN_LEN, MAVLINK_MSG_ID_INA219_LEN, MAVLINK_MSG_ID_INA219_CRC);
#endif
}

#if MAVLINK_MSG_ID_INA219_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ina219_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float voltageIn, float currentIn, float powerIn, float voltageOut, float currentOut, float powerOut)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, voltageIn);
    _mav_put_float(buf, 12, currentIn);
    _mav_put_float(buf, 16, powerIn);
    _mav_put_float(buf, 20, voltageOut);
    _mav_put_float(buf, 24, currentOut);
    _mav_put_float(buf, 28, powerOut);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INA219, buf, MAVLINK_MSG_ID_INA219_MIN_LEN, MAVLINK_MSG_ID_INA219_LEN, MAVLINK_MSG_ID_INA219_CRC);
#else
    mavlink_ina219_t *packet = (mavlink_ina219_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->voltageIn = voltageIn;
    packet->currentIn = currentIn;
    packet->powerIn = powerIn;
    packet->voltageOut = voltageOut;
    packet->currentOut = currentOut;
    packet->powerOut = powerOut;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INA219, (const char *)packet, MAVLINK_MSG_ID_INA219_MIN_LEN, MAVLINK_MSG_ID_INA219_LEN, MAVLINK_MSG_ID_INA219_CRC);
#endif
}
#endif

#endif

// MESSAGE INA219 UNPACKING


/**
 * @brief Get field timestamp from ina219 message
 *
 * @return  Time since system start
 */
static inline uint64_t mavlink_msg_ina219_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field voltageIn from ina219 message
 *
 * @return [V] Voltage in volts
 */
static inline float mavlink_msg_ina219_get_voltageIn(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field currentIn from ina219 message
 *
 * @return [A] Current in amperes
 */
static inline float mavlink_msg_ina219_get_currentIn(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field powerIn from ina219 message
 *
 * @return [W] Power in watts
 */
static inline float mavlink_msg_ina219_get_powerIn(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field voltageOut from ina219 message
 *
 * @return [V] Voltage in volts
 */
static inline float mavlink_msg_ina219_get_voltageOut(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field currentOut from ina219 message
 *
 * @return [A] Current in amperes
 */
static inline float mavlink_msg_ina219_get_currentOut(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field powerOut from ina219 message
 *
 * @return [W] Power in watts
 */
static inline float mavlink_msg_ina219_get_powerOut(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Decode a ina219 message into a struct
 *
 * @param msg The message to decode
 * @param ina219 C-struct to decode the message contents into
 */
static inline void mavlink_msg_ina219_decode(const mavlink_message_t* msg, mavlink_ina219_t* ina219)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    ina219->timestamp = mavlink_msg_ina219_get_timestamp(msg);
    ina219->voltageIn = mavlink_msg_ina219_get_voltageIn(msg);
    ina219->currentIn = mavlink_msg_ina219_get_currentIn(msg);
    ina219->powerIn = mavlink_msg_ina219_get_powerIn(msg);
    ina219->voltageOut = mavlink_msg_ina219_get_voltageOut(msg);
    ina219->currentOut = mavlink_msg_ina219_get_currentOut(msg);
    ina219->powerOut = mavlink_msg_ina219_get_powerOut(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_INA219_LEN? msg->len : MAVLINK_MSG_ID_INA219_LEN;
        memset(ina219, 0, MAVLINK_MSG_ID_INA219_LEN);
    memcpy(ina219, _MAV_PAYLOAD(msg), len);
#endif
}
