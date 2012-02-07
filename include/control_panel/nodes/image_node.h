/******************************************************************************
 * image_node.h
 *
 * Author:      Matt Richard
 * Date:        Sept. 6, 2011
 * Description: ROS node that subscribes to a ROS Image. This node is designed
 * 				to received either a camera feed or a laser scan, after the
 *				laser scan has been converted to a ROS Image.
 *****************************************************************************/

#ifndef CONTROL_PANEL_IMAGE_NODE_H
#define CONTROL_PANEL_IMAGE_NODE_H

#include <QObject>
#include <QImage>
#include "ros/ros.h"
#include "ros/transport_hints.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include <cv.h>
#include "control_panel/globals.h"

namespace enc = sensor_msgs::image_encodings;


class ImageNode : public QObject
{
	Q_OBJECT

	public:
		ImageNode(ros::NodeHandle *nh_ptr);
		~ImageNode();
		void subscribe();
		void unsubscribe();
		void imageCallback(const sensor_msgs::ImageConstPtr &msg);
		void setTopic(const std::string &topic);
		std::string getTopic() const;

	signals:
		void frameReceived(const QImage &buffer);

	private:
		std::string topic_name;
		ros::NodeHandle *nh;
		image_transport::ImageTransport *it;
		image_transport::Subscriber image_sub;
};

#endif // CONTROL_PANEL_IMAGE_NODE_H
