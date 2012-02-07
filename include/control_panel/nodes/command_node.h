/******************************************************************************
 * command_node.h
 *
 * Author:      Scott K Logan
 * Date:        Jan 14, 2012
 * Description: ROS node for controlling a robot's commands.
 *****************************************************************************/

#ifndef CONTROL_PANEL_COMMAND_NODE_H
#define CONTROL_PANEL_COMMAND_NODE_H

#include <QObject>
#include <stdio.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>


class CommandNode : public QObject
{
	Q_OBJECT

	public:
		CommandNode(ros::NodeHandle *nh_ptr);

	public slots:
		bool callEmpty(const QString &topicName );

	private:
		ros::NodeHandle *nh;
};

#endif // CONTROL_PANEL_COMMAND_NODE_H
