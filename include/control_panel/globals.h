/******************************************************************************
 * globals.h
 *
 * Author:      Matt Richard
 * Date:        July 2011
 * Description: Global constants that are used by many classes.
 *****************************************************************************/

#ifndef CONTROL_PANEL_GLOBALS_H
#define CONTROL_PANEL_GLOBALS_H

#define PI 3.14159265358979323846

#include <string>


namespace Globals
{

const double TWO_PI = (2.0 * PI);
const double PI_OVER_TWO = (0.5 * PI);
const double PI_OVER_FOUR = (0.25 * PI);

const double RAD_TO_DEG = (180.0 / PI);
const double DEG_TO_RAG = (PI / 180.0);

const char DegreesSymbol = 176;


// Common topic names
const std::string DEFAULT_CAMERA_TOPIC = "camera/image_raw";
const std::string DEFAULT_CONTROL_TOPIC = "cmd_vel";
const std::string DEFAULT_DIAGNOSTIC_TOPIC = "diagnostics";
const std::string DEFAULT_GPS_TOPIC = "gps";
const std::string DEFAULT_IMU_TOPIC = "imu/data";
const std::string DEFAULT_JOINT_TOPIC = "joint_states";
const std::string DEFAULT_JOYSTICK_TOPIC = "joy";
const std::string DEFAULT_LASER_TOPIC = "scan";
const std::string DEFAULT_MAP_TOPIC = "map";
const std::string DEFAULT_ODOMETRY_TOPIC = "odom";
const std::string DEFAULT_RANGE_TOPIC = "sonar";


// Connection status with a robot
enum ConnectionStatus
{
	Disconnected,
	Connecting,
	Connected
};


enum RCMode
{
    Disabled,
    Keyboard,
    Joystick
};

// Status of a component received from the diagnostic node
enum DiagnosticStatus
{
    Ok,
    Warn,
    Error
};

}

#endif // CONTROL_PANEL_GLOBALS_H
