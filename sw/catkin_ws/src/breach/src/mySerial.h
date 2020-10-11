/*
 * more info about serial
 *
 * http://wjwwood.io/serial/doc/1.1.0/index.html
 *
 *
 */

#include <ros/ros.h>
#include <stdlib.h>
#include <serial/serial.h>

#include <breach/serial_msg.h>
#include <vector>

#define FIX16_ONE 0x00010000

enum pc_receive_state_t {
    PC_RX_WAIT_FOR_BEGIN = 1,
    PC_RX_WAIT_FOR_HEADER = 2,
    PC_RX_WAIT_FOR_MESSAGE = 3
};

enum msg_size_t {
    SIZE_CHAR = 1,
    SIZE_INT8 = 1,
    SIZE_INT16 = 2,
    SIZE_INT32 = 4,
    SIZE_INT64 = 8,
};

enum msg_type_t {
    TYPE_COMMAND = 0,
    TYPE_UINT8 = 1,
    TYPE_INT8 = 2,
    TYPE_UINT16  = 10,
    TYPE_INT16 = 11,
    TYPE_UINT32 = 12,
    TYPE_INT32 = 13,
    TYPE_UINT64 = 14,
    TYPE_INT64 = 15,
    TYPE_FLOAT32 = 20,
    TYPE_FLOAT64 = 21,
    TYPE_STRING = 30,
    TYPE_DOUBLE_INT32 = 50,
};

enum msg_id_t {
    MESS_RX_DISTANCE_RESET = 1,     // reset encoders values, TYPE_COMMAND
    MESS_RX_SPEED_X = 5,            // set speed, TYPE_INT32, [mm*s(-1)]
    MESS_RX_SPEED_Y = 6,            // set speed, TYPE_INT32, [mm*s(-1)]
    MESS_RX_SPEED_Z = 7,            // set speed, TYPE_INT32, [rad*s(-1)]
    MESS_RX_PID_P = 100,            // TYPE_UINT32, [constant/1000]
    MESS_RX_PID_I = 101,            // TYPE_UINT32, [constant/1000]
    MESS_RX_PID_D = 102,            // TYPE_UINT32, [constant/1000]
    MESS_RX_SET_LIGHT = 110,        // TYPE_UINT16, [0 - 1000]

    MESS_TX_RADIUS = 4,             // get radius, TYPE_INT32, [mm]
    MESS_TX_SPEED_X = 5,            // get speed, TYPE_INT32, [mm*s(-1)]
    MESS_TX_SPEED_Y = 6,            // get speed, TYPE_INT32, [mm*s(-1)]
    MESS_TX_SPEED_Z = 7,            // get speed, TYPE_INT32, [rad*s(-1)]
    MESS_TX_SPEED_X_GOAL = 40,      // get speed goal, TYPE_INT32, [mm*s(-1)]
    MESS_TX_SPEED_Z_GOAL = 41,      // get speed goal, TYPE_INT32, [rad*s(-1)]
    MESS_TX_ENC_LEFT_SPEED = 50,    // get left wheel speed, TYPE_INT32, [mm*s(-1)]
    MESS_TX_ENC_RIGHT_SPEED = 51,   // get right wheel speed, TYPE_INT32, [mm*s(-1)]
    MESS_TX_ENC_LEFT_DISTANCE = 52, // get left wheel distance, TYPE_INT32, [mm]
    MESS_TX_ENC_RIGHT_DISTANCE = 53,// get right wheel distance, TYPE_INT32, [mm]
    // dual commands
    MESS_TX_ENC_SPEED = 60,         // TYPE_DOUBLE_INT32, [mm*s(-1)]
    MESS_TX_ENC_DISTANCE = 61,      // TYPE_DOUBLE_INT32, [mm]
    MESS_TX_SPEED_GOAL = 62,        // TYPE_DOUBLE_INT32, [mm*s(-1)]
    MESS_TX_MOTOR_CURRENT = 63,     // TYPE_DOUBLE_INT32, [mA]
    MESS_TX_MOTOR_VALUES = 64,      // TYPE_DOUBLE_INT32, [-32650;32650]
    // PID
    MESS_TX_PID_P = 100,            // TYPE_UINT32, [constant/1000]
    MESS_TX_PID_I = 101,            // TYPE_UINT32, [constant/1000]
    MESS_TX_PID_D = 102,            // TYPE_UINT32, [constant/1000]
    // 140: battery
    MESS_TX_BATTERY_MAIN = 140,     // TYPE_UINT16 [voltage/10]
    MESS_TX_BATTERY_STATUS = 149,   // TYPE_UINT8 [0: OK, 10: main battery low]
    // 150: bumpers
    MESS_TX_BUMPER_FRONT = 150,         // TYPE_UINT16, [0-65536]
    MESS_TX_BUMPER_REAR = 151,          // TYPE_UINT16, [0-65536]
    MESS_TX_BUMPER_FRONT_LEFT = 152,    // TYPE_UINT16, [0-65536]
    MESS_TX_BUMPER_FRONT_RIGHT = 153,   // TYPE_UINT16, [0-65536]
    MESS_TX_BUMPER_REAR_LEFT = 154,     // TYPE_UINT16, [0-65536]
    MESS_TX_BUMPER_REAR_RIGHT = 155,    // TYPE_UINT16, [0-65536]
    MESS_TX_E_STOP_STATUS = 158,        // TYPE_UINT8
                                        // [0x00=OK] [0x01=main ESTOP pressed] [0x10=second ESTOP pressed]
    MESS_TX_BUMPER_STATUS = 159,        // TYPE_UINT8
                                        // [0x00=OK] [0x01=front hit detected] [0x10=rear hit detected] [0x11=both hits detected]
    // 160: environmental sensors
    MESS_TX_ENVIRONMENT_TEMP = 160, // TYPE_UINT16 [degree/10]
    MESS_TX_ENVIRONMENT_HUM = 161,  // TYPE_UINT16 [value/10]
    // 190: internal states
    MESS_TX_CONTROL_LOOP_DUR = 190, // TYPE_UINT16 [ms]
    MESS_TX_SYSTEM_START = 191,     // low-level system state, TYPE_UINT8, [100: system is after reboot]
    MESS_TX_ROBOCLAW_TEMP = 192,    // temperature of Roboclaw °C/10
    // 200: sonars
    MESS_TX_SONAR_01 = 200,         // TYPE_UINT16, [mm]
    MESS_TX_SONAR_02 = 201,         // TYPE_UINT16, [mm]
    MESS_TX_SONAR_03 = 202,         // TYPE_UINT16, [mm]
    MESS_TX_SONAR_04 = 203,         // TYPE_UINT16, [mm]
    MESS_TX_SONAR_05 = 204,         // TYPE_UINT16, [mm]
    MESS_TX_SONAR_06 = 205,         // TYPE_UINT16, [mm]
    MESS_TX_SONAR_07 = 206,         // TYPE_UINT16, [mm]
    MESS_TX_SONAR_08 = 207,         // TYPE_UINT16, [mm]
    MESS_TX_SONAR_09 = 208,         // TYPE_UINT16, [mm]
    MESS_TX_SONAR_10 = 209,         // TYPE_UINT16, [mm]
    // 220: IRs
    MESS_TX_IR_1 = 220,             // TYPE_UINT16, [mm]
    MESS_TX_IR_2 = 221,             // TYPE_UINT16, [mm]
    MESS_TX_IR_3 = 222,             // TYPE_UINT16, [mm]
    MESS_TX_IR_4 = 223,             // TYPE_UINT16, [mm]
    MESS_TX_IR_5 = 224,             // TYPE_UINT16, [mm]
    MESS_TX_IR_ENABLED = 229,       // TYPE_UINT8   [0: disabled, 1: enabled]
};


class Message
{
    public:
        int         id;
        int         mess_len;
        int         data_type;
        int         data_char;
        int32_t     data_int;
        int32_t     data_int2;
        uint32_t    data_uint;
        int64_t     data_long;
        uint64_t    data_ulong;
        float       data_float;
        Message();
        Message(int o);
        ~Message();
        //std::vector<uint8_t> data;
        //~Message(int o);
};

class MySerial
{
public:
    bool pack_uint8(uint8_t value, uint8_t *b);
    bool pack_sint8(int8_t value, uint8_t *b);
    bool pack_uint16(uint16_t value, uint8_t *b);
    bool pack_sint16(int16_t value, uint8_t *b);
    bool pack_uint32(uint32_t value, uint8_t *b);
    bool pack_sint32(int32_t value, uint8_t *b);
    bool pack_uint64(uint64_t value, uint8_t *b);
    bool pack_sint64(int64_t value, uint8_t *b);
    bool pack_float32(float value, uint8_t *b);

    uint8_t unpack_uint8(uint8_t *b);
    int8_t unpack_sint8(uint8_t *b);
    uint16_t unpack_uint16(uint8_t *b);
    int16_t unpack_sint16(uint8_t *b);
    uint32_t unpack_uint32(uint8_t *b);
    int32_t unpack_sint32(uint8_t *b);
    uint64_t unpack_uint64(uint8_t *b);
    int64_t unpack_sint64(uint8_t *b);
    float unpack_float32(uint8_t *b);

private:
    char    c[13];
    int     mess_len;
    char    data[8];
    int     checksum;

public:
    serial::Serial dev;
    MySerial(std::string port, int baud, int timeout);

    Message mess;

    int receive(uint8_t *cc);
    void send(breach::serial_msg mess);

    bool send(uint8_t value, uint8_t id);
    bool send(int8_t value, uint8_t id);
    bool send(uint16_t value, uint8_t id);
    bool send(int16_t value, uint8_t id);
    bool send(uint32_t value, uint8_t id);
    bool send(int32_t value, uint8_t id);
    bool send(uint64_t value, uint8_t id);
    bool send(int64_t value, uint8_t id);
    bool send(char *value, uint8_t length, uint8_t id);
    bool send(int32_t value1, int32_t value2, uint8_t id);


    Message getData();
};



