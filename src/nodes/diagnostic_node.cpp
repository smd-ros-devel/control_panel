/******************************************************************************
** diagnostic_node.cpp
**
** Author:      Matt Richard
** Date:        Sept 8, 2011
** Description: ROS node for received robot diagnostic data.
******************************************************************************/

#include "control_panel/nodes/diagnostic_node.h"

/******************************************************************************
** Function:    DiagnosticNode
** Author:      Matt Richard
** Parameters:  ros::NodeHandle *nh_ptr
** Returns:     None
** Description: Constructor.
******************************************************************************/
DiagnosticNode::DiagnosticNode(ros::NodeHandle *nh_ptr)
{
	topic_name = Globals::DEFAULT_DIAGNOSTIC_TOPIC;

	nh = nh_ptr;

	strncpy(ns, &nh->getNamespace().c_str()[1], nh->getNamespace().length());
}

/******************************************************************************
** Function:    subscribe
** Author:      Matt Richard
** Parameters:  None
** Returns:     void
** Description:
******************************************************************************/
void DiagnosticNode::subscribe()
{
	diagnostic_sub = nh->subscribe(topic_name, 1,
		&DiagnosticNode::diagnosticCallback, this);
	std::cout << "Diagnostics: Subscribed to " << topic_name << std::endl;
}

/******************************************************************************
** Function:    unsubscribe
** Author:      Matt Richard
** Parameters:  None
** Returns:     void
** Description:
******************************************************************************/
void DiagnosticNode::unsubscribe()
{
	diagnostic_sub.shutdown();
}

/******************************************************************************
** Function:    diagnosticCallback
** Author:      Matt Richard
** Parameters:  const diagnostic_msgs::DiagnosticArrayConstPtr &msg -
** Returns:     void
** Description:
******************************************************************************/
void DiagnosticNode::diagnosticCallback(
	const diagnostic_msgs::DiagnosticArrayConstPtr &msg)
{
	for(unsigned int i = 0; i < msg->status.size(); i++)
	{
		// Process each diagnostic in the message
		// Only process messages from our bot
		if(!strncmp(msg->status[i].name.c_str(), ns, strlen(ns)))
		{
			//std::cout << "Diagnostics: Got: " << msg->status[i].name << " - " << msg->status[i].message << " - " << msg->status[i].hardware_id << std::endl;
			// Process each value in the diagnostic
			for(unsigned int j = 0; j < msg->status[i].values.size(); j++)
			{
				emit diagnosticDataReceived(msg->status[i].values[j].key.c_str(), msg->status[i].values[j].value.c_str());
				//std::cout << "Diagnostics: Recieved " << msg->status[i].values[j].key << " at " << msg->status[i].values[j].value << std::endl;
			}
		}
	}
}

/******************************************************************************
** Function:    setTopic
** Author:      Matt Richard
** Parameters:  std::string topic -
** Returns:     void
** Description:
******************************************************************************/
void DiagnosticNode::setTopic(const std::string &topic)
{
	topic_name = topic;
}

/******************************************************************************
** Function:    getTopic
** Author:      Matt Richard
** Parameters:  None
** Returns:     std::string -
** Description: 
******************************************************************************/
std::string DiagnosticNode::getTopic() const
{
	return topic_name;
}
