/******************************************************************************
 * command_node.cpp
 *
 * Author:      Scott K Logan
 * Date:        Jan 14, 2012
 * Description: ROS node for controlling a robot's commands.
 *****************************************************************************/
#include "control_panel/nodes/command_node.h"


CommandNode::CommandNode(ros::NodeHandle *nh_ptr)
{
	nh = nh_ptr;
}

/******************************************************************************
 * Public Slots
 *****************************************************************************/

bool CommandNode::callEmpty( const QString &topicName )
{
	std_srvs::Empty::Request req;
	std_srvs::Empty::Response res;
	ros::ServiceClient client = nh->serviceClient<std_srvs::Empty::Request>(topicName.toStdString());
	std::cout << "Calling " << topicName.toStdString() << std::endl;
	return client.call(req, res);
}

