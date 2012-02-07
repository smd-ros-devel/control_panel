/******************************************************************************
** joint_node.h
**
** Author:      Matt Richard
** Date:        Dec 7, 2011
** Description: ROS node for receiving joint data.
******************************************************************************/

#ifndef CONTROL_PANEL_JOINT_NODE_H
#define CONTROL_PANEL_JOINT_NODE_H

#include <QObject>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <stdio.h>
#include <string>
#include "control_panel/globals.h"


class JointNode : public QObject
{
	Q_OBJECT

	public:
		JointNode(ros::NodeHandle *nh_ptr);
		void subscribe();
		void unsubscribe();
		void jointCallback(const sensor_msgs::JointStateConstPtr &msg);
		void setTopic(const std::string &topic);
		std::string getTopic() const;

	signals:
		void jointDataReceived();

	private:
		std::string topic_name;
		ros::NodeHandle *nh;
		ros::Subscriber joint_sub;
};

#endif // CONTROL_PANEL_JOINT_NODE_H
