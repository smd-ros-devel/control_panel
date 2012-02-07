/* @todo Add license here */

/**
 * \file   joystick_node.h
 * \date   October 2011
 * \author Scott K Logan, Matt Richard
 * \brief  SRS Basestation interface to joystick_driver stack.
 */
#ifndef CONTROL_PANEL_JOYSTICK_NODE_H
#define CONTROL_PANEL_JOYSTICK_NODE_H

#include <QObject>
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "control_panel/globals.h"
#include <string>


/**
 * \class JoystickNode
 * \brief ROS node that receives a sensor_msgs::Joy message
 */
class JoystickNode : public QObject
{
	Q_OBJECT

	public:
		JoystickNode(ros::NodeHandle *nh_ptr);
        /**
         * \brief Empty deconstructor
         */
		~JoystickNode() { }

        /**
         * \brief Returns the topic name this node subscribes to
         */
        std::string getTopic() const { return joy_topic_name; }

        /**
         * \brief Emits a signal of all button and axis values that have changed
         *
         * \param msg The sensor_msgs::Joy message received
         */
		void joyCallback(const sensor_msgs::JoyConstPtr &msg);

        /**
         * \brief Sets the topic name for which this node should subscribe to
         *
         * \param topic_name The new topic to subscribe to
         */
        void setTopic(const std::string &topic_name) { joy_topic_name = topic_name; }

        /**
         * \brief Subscribes on the set topic
         */
        void subscribe();

        /**
         * \brief Shuts down the subscriber
         */
        void unsubscribe();

	signals:
        /**
         * \brief Emitted when the value of a joystick axis has changed
         */
		void axis_event(int, double);

        /**
         * \brief Emitted when the value of a joystick button has changed
         */
		void button_event(int, bool);

	private:
		std::string joy_topic_name;
		ros::NodeHandle *nh;
		ros::Subscriber joy_sub;
};

#endif // CONTROL_PANEL_JOYSTICK_NODE_H

