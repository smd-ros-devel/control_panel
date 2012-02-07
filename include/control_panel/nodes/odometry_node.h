/******************************************************************************
 * odometry_node.h
 *
 * Author:      Matt Richard
 * Date:        Aug 12, 2011
 * Description: ROS node for receiving odometry data.
 *****************************************************************************/

#ifndef CONTROL_PANEL_ODOMETRY_NODE_H
#define CONTROL_PANEL_ODOMETRY_NODE_H

#include <QObject>
#include <QVector3D>
#include <QQuaternion>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <stdio.h>
#include <string>
#include "control_panel/globals.h"


class OdometryNode : public QObject
{
	Q_OBJECT

	public:
		OdometryNode(ros::NodeHandle *nh_ptr);
		void subscribe();
		void unsubscribe();
		void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
		void setTopic(const std::string &topic);
		std::string getTopic() const;

	signals:
		void odometryDataReceived(const QVector3D &position,
            const QQuaternion &orientation, const QVector3D &linear_velocity,
            const QVector3D &angular_velocity);

	private:
		std::string topic_name;
		ros::NodeHandle *nh;
		ros::Subscriber odometry_sub;
};

#endif // CONTROL_PANEL_ODOMETRY_NODE_H
