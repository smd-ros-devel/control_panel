/******************************************************************************
 * control_node.h
 *
 * Author:      Matt Richard
 * Date:        Aug 31, 2011
 * Description: ROS node for controlling a robot's movement. The node receives
 * 				a command (keyboard or joystick) and translates it into linear
 *				and angular x, y, and z velocity.
 *****************************************************************************/

#ifndef CONTROL_PANEL_CONTROL_NODE_H
#define CONTROL_PANEL_CONTROL_NODE_H

#include <QObject>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <stdio.h>
#include <string>
#include "control_panel/globals.h"


class ControlNode : public QObject
{
	Q_OBJECT

	public:
		ControlNode(ros::NodeHandle *nh_ptr);
		void advertise();
		double getAngularX() const;
		double getAngularY() const;
		double getAngularZ() const;
		double getLinearX() const;
		double getLinearY() const;
		double getLinearZ() const;
		double getScale() const;
        std::string getTopic() const;
		geometry_msgs::Twist getTwist() const;
		//void publish();
        void setAngularX(double x);
        void setAngularY(double y);
        void setAngularZ(double z);
        void setLinearX(double x);
        void setLinearY(double y);
        void setLinearZ(double z);
		void setScale(double s);
		void setTopic(const std::string &topic);
        void unadvertise();

	public slots:
        void publish();
        void setAngularVector(double x = 0.0, double y = 0.0, double z = 0.0);
        void setLinearVector(double x = 0.0, double y = 0.0, double z = 0.0);
		void setTwist(const geometry_msgs::Twist &twist);
        void setTwist(double lx = 0.0, double ly = 0.0, double lz = 0.0, 
                      double ax = 0.0, double ay = 0.0, double az = 0.0);

	private:
		bool validVelocity(double value);

		std::string topic_name;
		ros::NodeHandle *nh;
		ros::Publisher control_pub;
		geometry_msgs::Twist twist_msg;
		double scale;
};

#endif // CONTROL_PANEL_CONTROL_NODE_H
