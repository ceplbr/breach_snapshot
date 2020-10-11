 #include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <ros/console.h>

#include <stdio.h>
#include <serial/serial.h>
#include "mySerial.h"

#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/LaserScan.h>

#include <breach/serial_msg.h>

#include <vector>
#include <string>

using namespace std;

//---------------------------------------------------------------------
// using serial
//   https://github.com/wjwwood/serial
//   http://wjwwood.io/serial/doc/1.1.0/classserial_1_1_serial.html
//
//   instal command: sudo apt install ros-DISTRO-serial
//
//---------------------------------------------------------------------

#define MBED_ADDR "/dev/usbSTM"

#define BAUD 115200
#define TIMEOUT 50

#define CONNECTION_TRIES 15

// distance, when sonar are included in local planner [m]
#define SRF_LPLNR_DIST 0.4

geometry_msgs::Twist cmd_request;
geometry_msgs::Twist cmd_recovery;
geometry_msgs::Twist cmd_vel_speed;
geometry_msgs::Twist pid;
geometry_msgs::Twist dist;

std_msgs::Float32 temperature_roboclaw;
std_msgs::Float32 voltage;
std_msgs::Float32 radius;
std_msgs::Float32 loop_duration;

std_msgs::Float32 motor_left_current;
std_msgs::Int32 motor_left_distance;
std_msgs::Float32 motor_left_speed;
std_msgs::Float32 motor_left_speed_req;
std_msgs::Int32 motor_left_value;

std_msgs::Float32 motor_right_current;
std_msgs::Int32 motor_right_distance;
std_msgs::Float32 motor_right_speed;
std_msgs::Float32 motor_right_speed_req;
std_msgs::Int32 motor_right_value;

std_msgs::Bool error_battery;
std_msgs::Bool error_bumperFront;
std_msgs::Bool error_bumperRear;
std_msgs::Bool error_estop;

sensor_msgs::Range      srf01;
sensor_msgs::Range      srf02;
sensor_msgs::Range      srf03;
sensor_msgs::Range      srf04;
sensor_msgs::Range      srf05;
sensor_msgs::Range      srf06;
sensor_msgs::Range      srf07;
sensor_msgs::Range      srf08;
sensor_msgs::Range      srf09;
sensor_msgs::Range      srf10;

sensor_msgs::LaserScan  SRF01;
sensor_msgs::LaserScan  SRF02;
sensor_msgs::LaserScan  SRF03;
sensor_msgs::LaserScan  SRF04;
sensor_msgs::LaserScan  SRF05;
sensor_msgs::LaserScan  SRF06;
sensor_msgs::LaserScan  SRF07;
sensor_msgs::LaserScan  SRF08;
sensor_msgs::LaserScan  SRF09;
sensor_msgs::LaserScan  SRF10;

std_msgs::Float32 ir1;
std_msgs::Float32 ir2;
std_msgs::Float32 ir3;
std_msgs::Float32 ir4;
std_msgs::Float32 ir5;

bool sendVel = false;
bool sendPID = false;
bool sendJoy = false;
bool sendRemote = false;
bool resetEncoders = false;
bool recoveryFlag = false;
bool sendFall = false;

std::string srf_config_param;
std::string ir_config_param;

bool srf01_enabled = false;
bool srf02_enabled = false;
bool srf03_enabled = false;
bool srf04_enabled = false;
bool srf05_enabled = false;
bool srf06_enabled = false;
bool srf07_enabled = false;
bool srf08_enabled = false;
bool srf09_enabled = false;
bool srf10_enabled = false;

bool ir01_enabled = false;
bool ir02_enabled = false;
bool ir03_enabled = false;
bool ir04_enabled = false;
bool ir05_enabled = false;

std_msgs::Bool ir_enable_val;
std_msgs::Bool fall_protection_cmd;

// ----------------------------
//   local functions
// ----------------------------

unsigned char char2byte(char c)
{
    unsigned char cr = ( c < 0 ) ? c + 256 : c;
    return cr;
}

void initSRF(sensor_msgs::Range *sensor, string frame_id)
{
    sensor->radiation_type = 0;
    sensor->header.frame_id = frame_id;
    sensor->field_of_view = 0.4;
    sensor->min_range = 0.02;
    sensor->max_range = 6.00;
}

void initSRF_as_Laser(sensor_msgs::LaserScan *sensor, string frame_id)
{
    sensor->header.frame_id = frame_id;
    sensor->range_min = 0.02;
    sensor->range_max = 6.00;
    sensor->angle_min = -0.1;
    sensor->angle_max =  0.1;
    sensor->angle_increment = 0.1;
    sensor->ranges.resize(3);
}

// ----------------------------
//   callbacks
// ----------------------------
void joy_vel_callback(const sensor_msgs::Joy& serial_cmd)
{
    if (!serial_cmd.buttons[4]) {
        cmd_vel_speed.linear.x = serial_cmd.axes[1];
        cmd_vel_speed.linear.y = serial_cmd.axes[2];
        sendJoy = true;
    }
    if (serial_cmd.buttons[4]) {
        resetEncoders = true;
    }
}

void cmd_vel_callback(const geometry_msgs::Twist& serial_cmd)
{
    cmd_vel_speed.linear.x = serial_cmd.linear.x;
    cmd_vel_speed.angular.z = serial_cmd.angular.z;
    sendVel = true;
}

void pid_callback(const geometry_msgs::Twist& serial_cmd)
{
    pid.linear.x += serial_cmd.linear.x;
    pid.linear.y += serial_cmd.angular.z;
    sendPID = true;
}

void remote_joy_callback(const geometry_msgs::Twist& serial_cmd)
{
    cmd_vel_speed.linear.x = serial_cmd.linear.x;
    cmd_vel_speed.angular.z = serial_cmd.angular.z;
    sendRemote = true;
}

void recovery_callback(const std_msgs::Bool& data)
{
    ;
    //recoveryFlag = data.data;
}

void cmd_recovery_callback(const geometry_msgs::Twist& cmd)
{
    cmd_recovery = cmd;
    sendVel = true;
    recoveryFlag = true;
}

void fall_protection_callback(const std_msgs::Bool& data)
{
    sendFall = true;
    fall_protection_cmd.data = data.data;
}

// ------------------------------
//   main
// ------------------------------

int main(int argc, char** argv)
{
    // ROS init
    std_msgs::String msg;
    std_msgs::Float32 radius_actual;
    geometry_msgs::Twist speed_actual;
    Message mess;
    MySerial *mbed = NULL;

    breach::serial_msg data_in;
    breach::serial_msg data_out;

    ros::init(argc, argv, "serial");

    ros::NodeHandle n;

    n.getParam("/ultrasonic_sensors", srf_config_param);
    n.getParam("/ir_sensors", ir_config_param);

    if(srf_config_param.find("SRF01") != std::string::npos){srf01_enabled = true;}
    if(srf_config_param.find("SRF02") != std::string::npos){srf02_enabled = true;}
    if(srf_config_param.find("SRF03") != std::string::npos){srf03_enabled = true;}
    if(srf_config_param.find("SRF04") != std::string::npos){srf04_enabled = true;}
    if(srf_config_param.find("SRF05") != std::string::npos){srf05_enabled = true;}
    if(srf_config_param.find("SRF06") != std::string::npos){srf06_enabled = true;}
    if(srf_config_param.find("SRF07") != std::string::npos){srf07_enabled = true;}
    if(srf_config_param.find("SRF08") != std::string::npos){srf08_enabled = true;}
    if(srf_config_param.find("SRF09") != std::string::npos){srf09_enabled = true;}
    if(srf_config_param.find("SRF10") != std::string::npos){srf10_enabled = true;}

    if(ir_config_param.find("IR01") != std::string::npos){ir01_enabled = true;}
    if(ir_config_param.find("IR02") != std::string::npos){ir02_enabled = true;}
    if(ir_config_param.find("IR03") != std::string::npos){ir03_enabled = true;}
    if(ir_config_param.find("IR04") != std::string::npos){ir04_enabled = true;}
    if(ir_config_param.find("IR05") != std::string::npos){ir05_enabled = true;}

    ros::Subscriber sub_joy = n.subscribe("/joy", 5, joy_vel_callback);
    ros::Subscriber sub_cmd = n.subscribe("/cmd_vel", 5, cmd_vel_callback);
    ros::Subscriber sub_rcv = n.subscribe("/cmd_recovery", 5, cmd_recovery_callback);
    ros::Subscriber sub_pid = n.subscribe("/pid_change", 10, pid_callback);
    ros::Subscriber sub_rmt = n.subscribe("/joy_teleop/cmd_vel", 5, remote_joy_callback);
    ros::Subscriber sub_recover = n.subscribe("/breach/error/recovery", 5, recovery_callback);
    ros::Subscriber sub_fall_protection = n.subscribe("/breach/fall_protection_change", 5, fall_protection_callback);

    ros::Publisher pub_cmd = n.advertise<geometry_msgs::Twist>("cmd_requested", 10);
    ros::Publisher pub_distance = n.advertise<geometry_msgs::Twist>("breach/distance", 10);
    ros::Publisher pub_temperatureRoboclaw = n.advertise<std_msgs::Float32>("breach/temperature_roboclaw", 10);
    ros::Publisher pub_voltage = n.advertise<std_msgs::Float32>("breach/voltage", 10);
    ros::Publisher pub_radius = n.advertise<std_msgs::Float32>("breach/radius", 10);
    ros::Publisher pub_fall_protection = n.advertise<std_msgs::Bool>("breach/fall_protection_status", 10);

    ros::Publisher pub_diag_loopDuration = n.advertise<std_msgs::Float32>("breach/diagnostics/control_loop_duration", 10);

    ros::Publisher srf01_pub;
    ros::Publisher srf02_pub;
    ros::Publisher srf03_pub;
    ros::Publisher srf04_pub;
    ros::Publisher srf05_pub;
    ros::Publisher srf06_pub;
    ros::Publisher srf07_pub;
    ros::Publisher srf08_pub;
    ros::Publisher srf09_pub;
    ros::Publisher srf10_pub;

    ros::Publisher SRF01_pub;
    ros::Publisher SRF02_pub;
    ros::Publisher SRF03_pub;
    ros::Publisher SRF04_pub;
    ros::Publisher SRF05_pub;
    ros::Publisher SRF06_pub;
    ros::Publisher SRF07_pub;
    ros::Publisher SRF08_pub;
    ros::Publisher SRF09_pub;
    ros::Publisher SRF10_pub;

    ros::Publisher pub_ir1;
    ros::Publisher pub_ir2;
    ros::Publisher pub_ir3;
    ros::Publisher pub_ir4;
    ros::Publisher pub_ir5;

    if(srf01_enabled) {
        srf01_pub = n.advertise<sensor_msgs::Range>("ultrasounds/srf01", 10);
        SRF01_pub = n.advertise<sensor_msgs::LaserScan>("ultrasounds/SRF01", 10);
    }
    if(srf02_enabled) {
        srf02_pub = n.advertise<sensor_msgs::Range>("ultrasounds/srf02", 10);
        SRF02_pub = n.advertise<sensor_msgs::LaserScan>("ultrasounds/SRF02", 10);
    }
    if(srf03_enabled) {
        srf03_pub = n.advertise<sensor_msgs::Range>("ultrasounds/srf03", 10);
        SRF03_pub = n.advertise<sensor_msgs::LaserScan>("ultrasounds/SRF03", 10);
    }
    if(srf04_enabled) {
        srf04_pub = n.advertise<sensor_msgs::Range>("ultrasounds/srf04", 10);
        SRF04_pub = n.advertise<sensor_msgs::LaserScan>("ultrasounds/SRF04", 10);
    }
    if(srf05_enabled) {
        srf05_pub = n.advertise<sensor_msgs::Range>("ultrasounds/srf05", 10);
        SRF05_pub = n.advertise<sensor_msgs::LaserScan>("ultrasounds/SRF05", 10);
    }
    if(srf06_enabled) {
        srf06_pub = n.advertise<sensor_msgs::Range>("ultrasounds/srf06", 10);
        SRF06_pub = n.advertise<sensor_msgs::LaserScan>("ultrasounds/SRF06", 10);
    }
    if(srf07_enabled) {
        srf07_pub = n.advertise<sensor_msgs::Range>("ultrasounds/srf07", 10);
        SRF07_pub = n.advertise<sensor_msgs::LaserScan>("ultrasounds/SRF07", 10);
    }
    if(srf08_enabled) {
        srf08_pub = n.advertise<sensor_msgs::Range>("ultrasounds/srf08", 10);
        SRF08_pub = n.advertise<sensor_msgs::LaserScan>("ultrasounds/SRF08", 10);
    }
    if(srf09_enabled) {
        srf09_pub = n.advertise<sensor_msgs::Range>("ultrasounds/srf09", 10);
        SRF09_pub = n.advertise<sensor_msgs::LaserScan>("ultrasounds/SRF09", 10);
    }
    if(srf10_enabled) {
        srf10_pub = n.advertise<sensor_msgs::Range>("ultrasounds/srf10", 10);
        SRF10_pub = n.advertise<sensor_msgs::LaserScan>("ultrasounds/SRF10", 10);
    }

    if(ir01_enabled) {pub_ir1 = n.advertise<std_msgs::Float32>("infrasensors/ir_1", 10);}
    if(ir02_enabled) {pub_ir2 = n.advertise<std_msgs::Float32>("infrasensors/ir_2", 10);}
    if(ir03_enabled) {pub_ir3 = n.advertise<std_msgs::Float32>("infrasensors/ir_3", 10);}
    if(ir04_enabled) {pub_ir4 = n.advertise<std_msgs::Float32>("infrasensors/ir_4", 10);}
    if(ir05_enabled) {pub_ir5 = n.advertise<std_msgs::Float32>("infrasensors/ir_5", 10);}

    ros::Publisher pub_error_battery = n.advertise<std_msgs::Bool>("breach/error/battery", 10);
    ros::Publisher pub_error_bumperFront = n.advertise<std_msgs::Bool>("breach/error/bumper_front", 10);
    ros::Publisher pub_error_bumperRear = n.advertise<std_msgs::Bool>("breach/error/bumper_rear", 10);
    ros::Publisher pub_error_estop = n.advertise<std_msgs::Bool>("breach/error/estop", 10);

    ros::Publisher pub_motor_left_current = n.advertise<std_msgs::Float32>("breach/motor/left/current", 10);
    ros::Publisher pub_motor_left_distance = n.advertise<std_msgs::Int32>("breach/motor/left/distance", 10);
    ros::Publisher pub_motor_left_speed = n.advertise<std_msgs::Float32>("breach/motor/left/speed", 10);
    ros::Publisher pub_motor_left_speed_req = n.advertise<std_msgs::Float32>("breach/motor/left/speed_req", 10);
    ros::Publisher pub_motor_left_value = n.advertise<std_msgs::Int32>("breach/motor/left/actionValue", 10);

    ros::Publisher pub_motor_right_current = n.advertise<std_msgs::Float32>("breach/motor/right/current", 10);
    ros::Publisher pub_motor_right_distance = n.advertise<std_msgs::Int32>("breach/motor/right/distance", 10);
    ros::Publisher pub_motor_right_speed = n.advertise<std_msgs::Float32>("breach/motor/right/speed", 10);
    ros::Publisher pub_motor_right_speed_req = n.advertise<std_msgs::Float32>("breach/motor/right/speed_req", 10);
    ros::Publisher pub_motor_right_value = n.advertise<std_msgs::Int32>("breach/motor/right/actionValue", 10);

    initSRF(&srf01, "srf01");
    initSRF(&srf02, "srf02");
    initSRF(&srf03, "srf03");
    initSRF(&srf04, "srf04");
    initSRF(&srf05, "srf05");
    initSRF(&srf06, "srf06");
    initSRF(&srf07, "srf07");
    initSRF(&srf08, "srf08");
    initSRF(&srf09, "srf09");
    initSRF(&srf10, "srf10");

    initSRF_as_Laser(&SRF01, "SRF01");
    initSRF_as_Laser(&SRF02, "SRF02");
    initSRF_as_Laser(&SRF03, "SRF03");
    initSRF_as_Laser(&SRF04, "SRF04");
    initSRF_as_Laser(&SRF05, "SRF05");
    initSRF_as_Laser(&SRF06, "SRF06");
    initSRF_as_Laser(&SRF07, "SRF07");
    initSRF_as_Laser(&SRF08, "SRF08");
    initSRF_as_Laser(&SRF09, "SRF09");
    initSRF_as_Laser(&SRF10, "SRF10");

    // try to connect to Mbed device
    int errorCount = 0;
    bool tryConnect = true;
    while (tryConnect && ros::ok()) {
        try {
            ROS_INFO("Connecting to device %s... (attempt %d/%d)",MBED_ADDR, errorCount+1,CONNECTION_TRIES+1);
            mbed = new MySerial(MBED_ADDR, BAUD, TIMEOUT);
            mbed->dev.setTimeout(10,10,1,10,1);
            tryConnect = false;
        }
        catch ( serial::IOException ex) {
            errorCount++;
            switch (ex.getErrorNumber()) {
                case 13:
                    if (errorCount > CONNECTION_TRIES) {
                        ROS_ERROR("\nConnection error. Permission denied. \n");
                        return 0;
                    }
                    break;
                case 2:
                    if (errorCount > CONNECTION_TRIES) {
                        ROS_ERROR("\nConnection error. No connection. \n");
                        return 0;
                    }
                    break;
                default:
                    if (errorCount > CONNECTION_TRIES) {
                        ROS_ERROR("\nUnknown connection error. \n");
                        return 0;
                    }
                    break;
            }
        }
        ros::Duration(1.0).sleep();
    }

    // main loop
    ros::Rate loop_rate(1000);

    int32_t value_int;
    int32_t value_int2;
    uint32_t value_uint;
    int64_t value_long;
    uint64_t value_ulong;
    while(ros::ok())
    {
        uint8_t buf[99], buf2[4], size, len, data_type, check, id;
        uint32_t checksum;
        // ==================================
        // DATA RECEIVE
        // ==================================

        // read first byte 0xf0
        //ROS_INFO("begin...");
        size = mbed->dev.read(buf, 1);
        //ROS_INFO("New data, size: %d", size);
        if( size == 1 && char2byte(buf[0]) == 0xf0 ) {
            ros::Time scan_time = ros::Time::now();

            // read length
            size = mbed->dev.read(buf, 1);
            if (!size) {
                continue;
            }
            len = char2byte(buf[0]);

            // read data_type
            size = mbed->dev.read(buf, 1);
            if (!size) {
                continue;
            }
            data_type = char2byte(buf[0]);

            // read id
            size = mbed->dev.read(buf, 1);
            if (!size) {
                continue;
            }
            id = char2byte(buf[0]);

            // read data and checksum
            size = mbed->dev.read(buf, len+1);
            if (size != len+1) {
                continue;
            }
            check = char2byte(buf[len]);

            checksum = len + data_type + id;
            for (int i = 0; i < len; i++) {
                checksum += char2byte(buf[i]);
            }
            checksum %= 256;
            if (check != checksum) {
                ROS_WARN("ID %d: checksum does not match! checksum: %d", id, checksum);
                continue;
            }
            switch (data_type) {
                case TYPE_UINT8:
                    value_uint = mbed->unpack_uint8(buf);
                    ROS_DEBUG("ID %d: UINT8 RECEIVED! Unpack value: %d", id, value_uint);
                    break;
                case TYPE_INT8:
                    value_int = mbed->unpack_sint8(buf);
                    ROS_DEBUG("ID %d: INT8 RECEIVED! Unpack value: %d", id, value_int);
                    break;
                case TYPE_UINT16:
                    value_uint = mbed->unpack_uint16(buf);
                    ROS_DEBUG("ID %d: UINT16 RECEIVED! Unpack value: %d", id, value_uint);
                    break;
                case TYPE_INT16:
                    value_int = mbed->unpack_sint16(buf);
                    ROS_DEBUG("ID %d: INT16 RECEIVED! Unpack value: %d", id, value_int);
                    break;
                case TYPE_UINT32:
                    value_uint = mbed->unpack_uint32(buf);
                    ROS_DEBUG("ID %d: UINT32 RECEIVED! Unpack value: %d", id, value_uint);
                    break;
                case TYPE_INT32:
                    value_int = mbed->unpack_sint32(buf);
                    ROS_DEBUG("ID %d: INT32 RECEIVED! Unpack value: %d", id, value_int);
                    break;
                case TYPE_UINT64:
                    value_ulong = mbed->unpack_uint64(buf);
                    ROS_DEBUG("ID %d: UNT64 RECEIVED! Unpack value: %lu", id, value_ulong);
                    break;
                case TYPE_INT64:
                    value_long = mbed->unpack_sint64(buf);
                    ROS_DEBUG("ID %d: INT64 RECEIVED! Unpack value: %ld", id, value_long);
                    break;
                case TYPE_FLOAT32:
                    ROS_WARN("ID %d: FLOAT32 FORMAT NOT SUPORTED!", id);
                    break;
                case TYPE_STRING:
                    ROS_WARN("ID %d: STRING FORMAT NOT SUPORTED!", id);
                    break;
                case TYPE_DOUBLE_INT32:
                    buf2[0] = buf[4];
                    buf2[1] = buf[5];
                    buf2[2] = buf[6];
                    buf2[3] = buf[7];
                    value_int = mbed->unpack_sint32(buf);
                    value_int2 = mbed->unpack_sint32(buf2);
                    ROS_DEBUG("ID %d: DOUBLE INT32 RECEIVED! Unpack value 1: %d, unpack value 2: %d", id, value_int, value_int2);
                    break;
                default:
                    ROS_WARN("ID %d: UNKNOWN DATA TYPE!", id);
                    break;
            }

            switch (id) {
                case MESS_TX_RADIUS:
                    radius.data = ((float)value_int)/1000;
                    pub_radius.publish(radius);
                    break;
                // 40: speed goal
                case MESS_TX_SPEED_X_GOAL:
                    cmd_request.linear.x = ((float)value_int)/1000;
                    pub_cmd.publish(cmd_request);
                    break;
                case MESS_TX_SPEED_Z_GOAL:
                    cmd_request.angular.z = ((float)value_int)/1000;
                    pub_cmd.publish(cmd_request);
                    break;
                // 50: odometry
                case MESS_TX_ENC_LEFT_DISTANCE:
                    motor_left_distance.data = value_int;
                    pub_motor_left_distance.publish(motor_left_distance);
                    break;
                case MESS_TX_ENC_RIGHT_DISTANCE:
                    motor_right_distance.data = value_int;
                    pub_motor_right_distance.publish(motor_right_distance);
                    break;
                case MESS_TX_ENC_LEFT_SPEED:
                    motor_left_speed.data = ((float)value_int)/1000;
                    pub_motor_left_speed.publish(motor_left_speed);
                    break;
                case MESS_TX_ENC_RIGHT_SPEED:
                    motor_right_speed.data = ((float)value_int)/1000;
                    pub_motor_right_speed.publish(motor_right_speed);
                    break;
                // 60: dual values
                case MESS_TX_ENC_SPEED:
                    motor_left_speed.data = ((float)value_int)/1000;
                    motor_right_speed.data = ((float)value_int2)/1000;
                    pub_motor_left_speed.publish(motor_left_speed);
                    pub_motor_right_speed.publish(motor_right_speed);
                    break;
                case MESS_TX_ENC_DISTANCE:
                    motor_left_distance.data = value_int;
                    motor_right_distance.data = value_int2;
                    dist.linear.x = motor_left_distance.data;
                    dist.linear.y = motor_right_distance.data;
                    pub_motor_left_distance.publish(motor_left_distance);
                    pub_motor_right_distance.publish(motor_right_distance);
                    pub_distance.publish(dist);
                    break;
                case MESS_TX_SPEED_GOAL:
                    motor_left_speed_req.data = ((float)value_int)/1000;
                    motor_right_speed_req.data = ((float)value_int2)/1000;
                    pub_motor_left_speed_req.publish(motor_left_speed_req);
                    pub_motor_right_speed_req.publish(motor_right_speed_req);
                    break;
                case MESS_TX_MOTOR_CURRENT:
                    motor_left_current.data = ((float)value_int)/1000;
                    motor_right_current.data = ((float)value_int)/1000;
                    pub_motor_left_current.publish(motor_left_current);
                    pub_motor_right_current.publish(motor_right_current);
                    break;
                case MESS_TX_MOTOR_VALUES:
                    motor_left_value.data = value_int;
                    motor_right_value.data = value_int2;
                    pub_motor_left_value.publish(motor_left_value);
                    pub_motor_right_value.publish(motor_right_value);
                    break;
                // 140: battery
                case MESS_TX_BATTERY_MAIN:
                    voltage.data = ((float)value_uint)/10;
                    pub_voltage.publish(voltage);
                    break;
                case MESS_TX_BATTERY_STATUS:
                    if(value_int == 10) {
                        error_battery.data = true;
                    } else {
                        error_battery.data = false;
                    }
                    pub_error_battery.publish(error_battery);
                    break;
                // 150: bumpers, estop
                case MESS_TX_E_STOP_STATUS:
                    if(value_int) {
                        error_estop.data = true;
                    } else {
                        error_estop.data = false;
                    }
                    pub_error_estop.publish(error_estop);
                    break;
                case MESS_TX_BUMPER_STATUS:
                    if(value_int & 0x10) {
                        error_bumperRear.data = true;
                    } else {
                        error_bumperRear.data = false;
                    }

                    if(value_int & 0x01) {
                        error_bumperFront.data = true;
                    } else {
                        error_bumperFront.data = false;
                    }

                    pub_error_bumperFront.publish(error_bumperFront);
                    pub_error_bumperRear.publish(error_bumperRear);
                    break;
                // 190: internal states
                case MESS_TX_CONTROL_LOOP_DUR:
                    loop_duration.data = ((float)value_int)/1000;
                    pub_diag_loopDuration.publish(loop_duration);
                    break;
                case MESS_TX_ROBOCLAW_TEMP:
                    temperature_roboclaw.data = ((float)value_int)/10;
                    pub_temperatureRoboclaw.publish(temperature_roboclaw);
                    break;
                // 200: sonars
                case MESS_TX_SONAR_01:
                    if(srf01_enabled) {
                        srf01.range = ((float)value_uint)/1000 ;
                        srf01_pub.publish(srf01);
                        SRF01.header.stamp = scan_time;
                        for (int i = 0; i < 3; i++) {
                            SRF01.ranges[i] = (((float)value_uint)/1000 > SRF_LPLNR_DIST) ? 5.0 : ((float)value_uint)/1000 ;
                        }
                        SRF01_pub.publish(SRF01);
                    }
                    break;
                case MESS_TX_SONAR_02:
                    if(srf02_enabled) {
                        srf02.range = ((float)value_uint)/1000 ;
                        srf02_pub.publish(srf02);
                        SRF02.header.stamp = scan_time;
                        for (int i = 0; i < 3; i++) {
                            SRF02.ranges[i] = (((float)value_uint)/1000 > SRF_LPLNR_DIST) ? 5.0 : ((float)value_uint)/1000 ;
                        }
                        SRF02_pub.publish(SRF02);
                    }
                    break;
                case MESS_TX_SONAR_03:
                    if(srf03_enabled) {
                        srf03.range = ((float)value_uint)/1000 ;
                        srf03_pub.publish(srf03);
                        SRF03.header.stamp = scan_time;
                        for (int i = 0; i < 3; i++) {
                            SRF03.ranges[i] = (((float)value_uint)/1000 > SRF_LPLNR_DIST) ? 5.0 : ((float)value_uint)/1000 ;
                        }
                        SRF03_pub.publish(SRF03);
                    }
                    break;
                case MESS_TX_SONAR_04:
                    if(srf04_enabled) {
                        srf04.range = ((float)value_uint)/1000 ;
                        srf04_pub.publish(srf04);
                        SRF04.header.stamp = scan_time;
                        for (int i = 0; i < 3; i++) {
                            SRF04.ranges[i] = (((float)value_uint)/1000 > SRF_LPLNR_DIST) ? 5.0 : ((float)value_uint)/1000 ;
                        }
                        SRF04_pub.publish(SRF04);
                    }
                    break;
                case MESS_TX_SONAR_05:
                    if(srf05_enabled) {
                        srf05.range = ((float)value_uint)/1000 ;
                        srf05_pub.publish(srf05);
                        SRF05.header.stamp = scan_time;
                        for (int i = 0; i < 3; i++) {
                            SRF05.ranges[i] = (((float)value_uint)/1000 > SRF_LPLNR_DIST) ? 5.0 : ((float)value_uint)/1000 ;
                        }
                        SRF05_pub.publish(SRF05);
                    }
                    break;
                case MESS_TX_SONAR_06:
                    if(srf06_enabled) {
                        srf06.range = ((float)value_uint)/1000 ;
                        srf06_pub.publish(srf06);
                        SRF06.header.stamp = scan_time;
                        for (int i = 0; i < 3; i++) {
                            SRF06.ranges[i] = (((float)value_uint)/1000 > SRF_LPLNR_DIST) ? 5.0 : ((float)value_uint)/1000 ;
                        }
                        SRF06_pub.publish(SRF06);
                    }
                    break;
                case MESS_TX_SONAR_07:
                    if(srf07_enabled) {
                        srf07.range = ((float)value_uint)/1000 ;
                        srf07_pub.publish(srf07);
                        SRF07.header.stamp = scan_time;
                        for (int i = 0; i < 3; i++) {
                            SRF07.ranges[i] = (((float)value_uint)/1000 > SRF_LPLNR_DIST) ? 5.0 : ((float)value_uint)/1000 ;
                        }
                        SRF07_pub.publish(SRF07);
                    }
                    break;
                case MESS_TX_SONAR_08:
                    if(srf08_enabled) {
                        srf08.range = ((float)value_uint)/1000 ;
                        srf08_pub.publish(srf08);
                        SRF08.header.stamp = scan_time;
                        for (int i = 0; i < 3; i++) {
                            SRF08.ranges[i] = (((float)value_uint)/1000 > SRF_LPLNR_DIST) ? 5.0 : ((float)value_uint)/1000 ;
                        }
                        SRF08_pub.publish(SRF08);
                    }
                    break;
                case MESS_TX_SONAR_09:
                    if(srf09_enabled) {
                        srf09.range = ((float)value_uint)/1000 ;
                        srf09_pub.publish(srf09);
                        SRF09.header.stamp = scan_time;
                        for (int i = 0; i < 3; i++) {
                            SRF09.ranges[i] = (((float)value_uint)/1000 > SRF_LPLNR_DIST) ? 5.0 : ((float)value_uint)/1000 ;
                        }
                        SRF09_pub.publish(SRF09);
                    }
                    break;
                case MESS_TX_SONAR_10:
                    if(srf10_enabled) {
                        srf10.range = ((float)value_uint)/1000 ;
                        srf10_pub.publish(srf10);
                        SRF10.header.stamp = scan_time;
                        for (int i = 0; i < 3; i++) {
                            SRF10.ranges[i] = (((float)value_uint)/1000 > SRF_LPLNR_DIST) ? 5.0 : ((float)value_uint)/1000 ;
                        }
                        SRF10_pub.publish(SRF10);
                    }
                    break;
                case MESS_TX_IR_1:
                    if(ir01_enabled) {
                        std_msgs::Float32 ir_val;
                        ir_val.data = ((float)value_uint)/1000;
                        pub_ir1.publish(ir_val);
                    }
                    break;
                case MESS_TX_IR_2:
                    if(ir02_enabled) {
                        std_msgs::Float32 ir_val;
                        ir_val.data = ((float)value_uint)/1000;
                        pub_ir2.publish(ir_val);
                    }
                    break;
                case MESS_TX_IR_3:
                    if(ir03_enabled) {
                        std_msgs::Float32 ir_val;
                        ir_val.data = ((float)value_uint)/1000;
                        pub_ir3.publish(ir_val);
                    }
                    break;
                case MESS_TX_IR_4:
                    if(ir04_enabled) {
                        std_msgs::Float32 ir_val;
                        ir_val.data = ((float)value_uint)/1000;
                        pub_ir4.publish(ir_val);
                    }
                    break;
                case MESS_TX_IR_5:
                    if(ir05_enabled) {
                        std_msgs::Float32 ir_val;
                        ir_val.data = ((float)value_uint)/1000;
                        pub_ir5.publish(ir_val);
                    }
                    break;
                case MESS_TX_IR_ENABLED:
                    //ROS_INFO("IR enabled: %u", value_uint);
                    ir_enable_val.data = (bool)value_uint;
                    pub_fall_protection.publish(ir_enable_val);
                    break;
                default:
                    ROS_WARN("ID %d: UNKNOWN ID!", id);
                    break;

            }
        }

        // ================================
        // DATA SEND
        // ================================
        if( resetEncoders ) {                 // cmd vel
            resetEncoders = false;

            mbed->send((uint8_t)(1), 1);
        }

        if( sendJoy ) {                 // cmd vel
            sendJoy = false;

            mbed->send((int32_t)(cmd_vel_speed.linear.x*600), 5);
            sleep(1000);
            mbed->send((int32_t)(cmd_vel_speed.linear.y*3000), 6);
        }

        if( sendVel ) {                 // cmd vel
            sendVel = false;

            if(!recoveryFlag) {
                mbed->send((int32_t)(cmd_vel_speed.linear.x*1000), 5);
                usleep(1000);
                mbed->send((int32_t)(cmd_vel_speed.angular.z*1000), 7);
            } else {
                ROS_WARN("RECOVERY MODE");
                mbed->send((int32_t)(cmd_recovery.linear.x*1000), 5);
                usleep(1000);
                mbed->send((int32_t)(cmd_recovery.angular.z*1000), 7);
                recoveryFlag = false;
            }
        }
        if( sendPID ) {           // PID change
            sendPID = false;

            data_out.id = 100;
            data_out.data_type = 3;
            data_out.length = 8;
            data_out.data_float = pid.linear.x;
            mbed->send( data_out );

            data_out.id = 101;
            data_out.data_type = 3;
            data_out.length = 8;
            data_out.data_float = pid.linear.y;
            mbed->send( data_out );
        }

        if( sendRemote ) {                 // cmd vel
            sendRemote = false;

            mbed->send((int32_t)(cmd_vel_speed.linear.x*500), 5);
            usleep(1000);
            mbed->send((int32_t)(cmd_vel_speed.angular.z*500), 7);
        }

        if( sendFall ) {                 // Fall protection
            sendFall = false;
            mbed->send((int8_t)(fall_protection_cmd.data), MESS_TX_IR_ENABLED);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}




