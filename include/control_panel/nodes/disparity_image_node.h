/* @todo Add license here. */

/**
 * \file   disparity_image_node.h
 * \date   Feb 5, 2012
 * \author Matt Richard
 */

#ifndef CONTROL_PANEL_DISPARITY_IMAGE_NODE_H
#define CONTROL_PANEL_DISPARITY_IMAGE_NODE_H

#include <QObject>
#include <QImage>
#include "ros/ros.h"
#include "sensor_msgs/image_encodings.h"
#include "stereo_msgs/DisparityImage.h"
#include <string>

namespace enc = sensor_msgs::image_encodings;

/**
 * \class DisparityImageNode
 * \brief Receives a disparity image and converts it to a QImage
 */
class DisparityImageNode : public QObject
{
    Q_OBJECT

    public:
        DisparityImageNode(ros::NodeHandle *nh_ptr);

        /**
         * \brief Converts the received disparity image into a QImage
         *
         * \param msg The received stereo_msgs::DisparityImage to convert
         */
        void disparityCallback(const stereo_msgs::DisparityImage::ConstPtr &msg);

        /**
         * \brief Returns the subscribers topic name
         */
        std::string getTopic() const { return topic_name; }

        /**
         * \brief Sets the topic name to topic
         *
         * \param topic The new topic name to subscribe over
         */
        void setTopic(const std::string &topic) { topic_name = topic; }

        /**
         * \brief Subscribes to the set topic name
         */
        void subscribe();

        /**
         * \brief Shuts down the disparity subscriber
         */
        void unsubscribe();

    signals:
        void disparityImageReceived(const QImage &image);

    private:
        std::string topic_name;
        ros::NodeHandle *nh;
        ros::Subscriber disparity_sub;
};

#endif // CONTROL_PANEL_DISPARITY_IMAGE_NODE_H
