/******************************************************************************
 * laser_node.h
 *
 * Author:      Matt Richard
 * Date:        Nov. 16, 2011
 * Description: ROS node for receiving a ROS LaserScan.msg message.
 *****************************************************************************/

#ifndef CONTROL_PANEL_LASER_NODE_H
#define CONTROL_PANEL_LASER_NODE_H

#include <QObject>
#include <QImage>
#include <QRgb>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <string>
#include <stdio.h>
#include <math.h>
#include "control_panel/globals.h"

/**
 * \class LaserNode
 * \brief Recieves a sensor_msgs::LaserScan and converts it to a QImage.
 *
 * \author Matt Richard
 */
class LaserNode : public QObject
{
	Q_OBJECT

	public:
		LaserNode(ros::NodeHandle *nh_ptr);

        /**
         * \brief Subscribes to the set topic
         */
		void subscribe();

        /**
         * \brief Unsubscribes from the current topic.
         */
        void unsubscribe();

        /**
         * \brief ROS callback function for the incoming laser scan.
         */
		void laserCallback(const sensor_msgs::LaserScanConstPtr &msg);

        /**
         * \brief Sets the topic on which to receive a laser scan.
         *
         * \param topic The topic to subscribe to.
         */
        void setTopic(const std::string &topic);

        /**
         * \return Returns the set topic
         */
		std::string getTopic() const;

	signals:
        /**
         * \brief Signal emitted after a laser scan has been received and converted
         *
         * \param buffer   The displayable laser scan image created
         * \param interval The max range of the laser scan (meters)
         */
		void laserScanReceived(const QImage &buffer, int interval);

	private:
		std::string topic_name;
		ros::NodeHandle *nh;
		ros::Subscriber laser_sub;

        QRgb white;
        QRgb red;
};

#endif // CONTROL_PANEL_LASER_NODE_H
