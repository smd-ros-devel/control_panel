/*****************************************************************************
** joystick_node.h
** 
** Author:      Scott K Logan
** Date:        October 2011
** Description: SRS Basestation interface to joystick_driver stack.
*****************************************************************************/

#ifndef CONTROL_PANEL_JOYSTICK_NODE_H
#define CONTROL_PANEL_JOYSTICK_NODE_H

#include <QThread>
#include <QVector>
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "control_panel/globals.h"


class JoystickNode : public QThread
{
	Q_OBJECT

	public:
		JoystickNode();
		~JoystickNode();
		bool isEnabled();
		void joyCallback(const sensor_msgs::JoyConstPtr &msg);
		void run();

	public slots:
		void enableJoystick();
		void disableJoystick();
		void stopJoystick();

	signals:
		void joystickMsgReceived(char *);
		//void velCmdReceived(double, double, double, double, double, double);
		void axis_event(int, double);
		void button_event(int, bool);

	private:
		bool useJoy;
		short unsigned int maxScale;
		std::string joy_topic_name;
		ros::NodeHandle *nh;
		ros::Subscriber joy_sub;
		geometry_msgs::Twist *cmd_vel;
};

#endif // CONTROL_PANEL_JOYSTICK_NODE_H

