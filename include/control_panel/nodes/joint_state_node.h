/* @todo Add license here */


/**
 * \file   joint_state_node.h
 * \date   Dec 7, 2011
 * \author Matt Richard
 */
#ifndef CONTROL_PANEL_JOINT_STATE_NODE_H
#define CONTROL_PANEL_JOINT_STATE_NODE_H

#include <QObject>
#include <QStringList>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <string>
#include <vector>
#include "control_panel/globals.h"

/**
 * \class JointStateNode
 * \brief ROS node to receive a sensor_msgs::JointState message from a robot
 */
class JointStateNode : public QObject
{
	Q_OBJECT

	public:
        /**
         * \brief Constructor. Initializes the topic name and copys the node handle pointer
         */
		JointStateNode(ros::NodeHandle *nh_ptr);

        /**
         * \brief Returns the subscriber's topic
         */
		std::string getTopic() const { return topic_name; }

        /**
         * \brief Callback function for when a sensor_msgs::JointState message is received
         *
         * \param msg sensor_msgs::JointState message received
         */
		void jointCallback(const sensor_msgs::JointStateConstPtr &msg);

        /**
         * \brief Sets the topic over which the subscriber should subscribe to.
         */
		void setTopic(const std::string &topic) { topic_name = topic; }

        /**
         * \brief Subscribes over the set topic
         */
        void subscribe();

        /**
         * \brief Shutsdown the subscriber
         */
        void unsubscribe();

	signals:
        /**
         * \brief Signal emitted after a message is received
         *
         * \param names The list of joint names
         * \param pos   Positions of each joint
         * \param vel   Velocities of each joint
         * \param eff   Effort of each joint
         */
		void jointDataReceived(const QStringList &names, const std::vector<double> &pos,
                               const std::vector<double> &vel, const std::vector<double> &eff);

	private:
		std::string topic_name;
		ros::NodeHandle *nh;
		ros::Subscriber joint_sub;
};

#endif // CONTROL_PANEL_JOINT_STATE_NODE_H
