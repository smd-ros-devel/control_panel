/* @todo Add license here */

/**
 * \file   joystick_node.cpp
 * \date   Oct, 2011
 * \author Scott K Logan, Matt Richard
 */
#include <QVector>
#include "control_panel/nodes/joystick_node.h"

JoystickNode::JoystickNode(ros::NodeHandle *nh_ptr) : nh(nh_ptr)
{
	joy_topic_name = Globals::DEFAULT_JOYSTICK_TOPIC;
}

void JoystickNode::joyCallback(const sensor_msgs::JoyConstPtr &msg)
{
	unsigned int i;

	static QVector<double> axes(msg->axes.size(), 0);
	static QVector<bool> buttons(msg->buttons.size(), false);

	if((unsigned)axes.size() != msg->axes.size())
		axes.fill(msg->axes.size(), 0);
	if((unsigned)buttons.size() != msg->buttons.size())
		buttons.fill(msg->buttons.size(), false);

	for(i = 0; i < msg->axes.size(); i++)
	{
		if(msg->axes[i] != axes[i])
		{
			axes[i] = msg->axes[i];
//			std::cout << "Axis " << i << ": " << msg->axes[i] << std::endl;
			emit axis_event(i, msg->axes[i]);
		}
	}

	for(i = 0; i < msg->buttons.size(); i++)
	{
		if(msg->buttons[i] != buttons[i])
		{
			buttons[i] = msg->buttons[i];
//			std::cout << "Buttons " << i << ": " << buttons[i] << std::endl;
			emit button_event(i, msg->buttons[i]);
		}
	}
}

void JoystickNode::subscribe()
{
    joy_sub = nh->subscribe(joy_topic_name, 1, &JoystickNode::joyCallback, this);
}

void JoystickNode::unsubscribe()
{
    joy_sub.shutdown();
}
