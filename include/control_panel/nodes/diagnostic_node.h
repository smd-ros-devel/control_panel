/******************************************************************************
** diagnostic_node.h
**
** Author:      Matt Richard
** Date:        Sept 8, 2011
** Description: ROS node for receiving robot diagnostic data.
******************************************************************************/

#ifndef CONTROL_PANEL_DIAGNOSTIC_NODE_H
#define CONTROL_PANEL_DIAGNOSTIC_NODE_H

#include <QObject>
#include "ros/ros.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include <stdio.h>
#include <string>
#include "control_panel/globals.h"


class DiagnosticNode : public QObject
{
	Q_OBJECT

	public:
		DiagnosticNode(ros::NodeHandle *nh_ptr);
		void subscribe();
		void unsubscribe();
		void diagnosticCallback(const diagnostic_msgs::DiagnosticArrayConstPtr &msg);
		void setTopic(const std::string &topic);
		std::string getTopic() const;

	signals:
		void diagnosticDataReceived(const QString &key, const QString &val);

	private:
		std::string topic_name;
		ros::NodeHandle *nh;
		ros::Subscriber diagnostic_sub;
		char ns[255];
};

#endif // CONTROL_PANEL_DIAGNOSTIC_NODE_H
