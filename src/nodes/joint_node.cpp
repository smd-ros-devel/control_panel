/******************************************************************************
** joint_node.cpp
**
** Author:      Matt Richard
** Date:        Dec 7, 2011
** Description:
******************************************************************************/

#include "control_panel/nodes/joint_node.h"

/******************************************************************************
** Function:    JointNode
** Author:      Matt Richard
** Parameters:  ros::NodeHandle *nh_ptr - 
** Returns:     None
** Description: Constructor.
******************************************************************************/
JointNode::JointNode(ros::NodeHandle *nh_ptr)
{
	topic_name = Globals::DEFAULT_JOINT_TOPIC;

	nh = nh_ptr;
}

/******************************************************************************
** Function:    subscribe
** Author:      Matt Richard
** Parameters:  None
** Returns:     void
** Description: 
******************************************************************************/
void JointNode::subscribe()
{
	joint_sub = nh->subscribe(topic_name, 1, &JointNode::jointCallback, this);
}

/******************************************************************************
** Function:    unsubscribe
** Author:      Matt Richard
** Parameters:  None
** Returns:     void
** Description:
******************************************************************************/
void JointNode::unsubscribe()
{
	joint_sub.shutdown();
}

/******************************************************************************
** Function:    jointCallback
** Author:      Matt Richard
** Parameters:  const sensor_msgs::JointStateConstPtr &msg - 
** Returns:     void
** Description:
******************************************************************************/
void JointNode::jointCallback(const sensor_msgs::JointStateConstPtr &msg)
{
}

/******************************************************************************
** Function:    setTopic
** Author:      Matt Richard
** Parameters:  std::string topic - 
** Returns:     void
** Description:
******************************************************************************/
void JointNode::setTopic(const std::string &topic)
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
std::string JointNode::getTopic() const
{
	return topic_name;
}
