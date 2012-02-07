/******************************************************************************
** imu_node.h
**
** Author:      Matt Richard
** Date:        Sept 8, 2011
** Description: ROS node for receiving imu data.
******************************************************************************/

#ifndef CONTROL_PANEL_IMU_NODE_H
#define CONTROL_PANEL_IMU_NODE_H

#include <QObject>
#include <QQuaternion>
#include <QVector3D>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <stdio.h>
#include <string>
#include "control_panel/globals.h"


class ImuNode : public QObject
{
	Q_OBJECT

	public:
		ImuNode(ros::NodeHandle *nh_ptr);
		void subscribe();
		void unsubscribe();
		void imuCallback(const sensor_msgs::ImuConstPtr &msg);
		void setTopic(const std::string &topic);
		std::string getTopic() const;

	signals:
		void imuDataReceived(const QQuaternion &ori, const QVector3D &ang_vel,
            const QVector3D &lin_accel);

	private:
		std::string topic_name;
		ros::NodeHandle *nh;
		ros::Subscriber imu_sub;
};

#endif // CONTROL_PANEL_IMU_NODE_H
