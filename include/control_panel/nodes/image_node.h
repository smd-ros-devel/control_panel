/* @todo Add license here */


/**
 * \file   image_node.h
 * \date   Sept 6, 2011
 * \author Matt Richard
 */
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


/**
 * \class ImageNode
 * \brief ROS node that receives a sensor_msgs::Image message and converts it to a QImage
 */
class ImageNode : public QObject
{
	Q_OBJECT

	public:
        /**
         * \brief Initializes the ImageTransport
         *
         * \param nh_ptr Then node handle to use for subscribing
         */
		ImageNode(ros::NodeHandle *nh_ptr);

        /**
         * \brief Deletes the ImageTransport to ensure we unsubscribe from the subscribed topic
         */
		~ImageNode();

        /**
         * \brief Returns the topic the subscriber subscribes to
         */
        std::string getTopic() const { return topic_name; }

        /**
         * \brief Callback function for when we receive a sensor_msgs::Image.msg
         *
         * This converts the sensor_msgs::Image into a QImage.
         *
         * \param msg The image received
         */
		void imageCallback(const sensor_msgs::ImageConstPtr &msg);

        /**
         * \brief Sets topic for which to subscribe to
         *
         * \param topic The topic to subscribe to
         */
		void setTopic(const std::string &topic) { topic_name = topic; }

        /**
         * \brief Starts the subscription over the set topic
         */
		void subscribe();

        /**
         * \brief Shuts down the subscriber
         */
        void unsubscribe();

	signals:
        /**
         * \brief Signal that is emitted once the image receive has been converted
         *
         * \param buffer The converted image buffer
         */
		void frameReceived(const QImage &buffer);

	private:
		std::string topic_name;
		ros::NodeHandle *nh;
		image_transport::ImageTransport *it;
		image_transport::Subscriber image_sub;
};

#endif // CONTROL_PANEL_IMAGE_NODE_H
