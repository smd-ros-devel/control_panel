/******************************************************************************
** gps_node.h
**
** Author:      Matt Richard
** Date:        Aug 12, 2011
** Description: ROS node for receiving gps data.
******************************************************************************/

#ifndef CONTROL_PANEL_GPS_NODE_H
#define CONTROL_PANEL_GPS_NODE_H

#include <QObject>
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include <stdio.h>
#include <string>
#include "control_panel/globals.h"


class GpsNode : public QObject
{
	Q_OBJECT

	public:
		GpsNode(ros::NodeHandle *nh_ptr);
		void subscribe();
		void unsubscribe();
		void gpsCallback(const sensor_msgs::NavSatFixConstPtr &msg);
		void setTopic(const std::string &topic);
		std::string getTopic() const;

	signals:
		void gpsDataReceived(double lat, double lon, double alt);

	private:
		std::string topic_name;
		ros::NodeHandle *nh;
		ros::Subscriber gps_sub;
};

#endif // CONTROL_PANEL_GPS_NODE_H
