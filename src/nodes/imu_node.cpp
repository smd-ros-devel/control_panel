/******************************************************************************
** imu_node.cpp
**
** Author:      Matt Richard
** Date:        Sept 8, 2011
** Description:
******************************************************************************/

#include "control_panel/nodes/imu_node.h"

/******************************************************************************
** Function:    ImuNode
** Author:      Matt Richard
** Parameters:  ros::NodeHandle *nh_ptr -
** Returns:     None
** Description: Constructor.
******************************************************************************/
ImuNode::ImuNode(ros::NodeHandle *nh_ptr)
{
	topic_name = Globals::DEFAULT_IMU_TOPIC;

	nh = nh_ptr;
}

/******************************************************************************
** Function:    subscribe
** Author:      Matt Richard
** Parameters:  None
** Returns:     void
** Description:
******************************************************************************/
void ImuNode::subscribe()
{
	imu_sub = nh->subscribe(topic_name, 1, &ImuNode::imuCallback, this);
}

/******************************************************************************
** Function:    unsubscribe
** Author:      Matt Richard
** Parameters:  None
** Returns:     void
** Description:
******************************************************************************/
void ImuNode::unsubscribe()
{
	imu_sub.shutdown();
}

/******************************************************************************
** Function:    imuCallback
** Author:      Matt Richard
** Parameters:  const sensor_msgs::ImuConstPtr &msg -
** Returns:     void
** Description:
******************************************************************************/
void ImuNode::imuCallback(const sensor_msgs::ImuConstPtr &msg)
{
	emit imuDataReceived(
        QQuaternion(msg->orientation.w,
                    msg->orientation.x,
                    msg->orientation.y,
                    msg->orientation.z),
        QVector3D(msg->angular_velocity.x,
                  msg->angular_velocity.y,
                  msg->angular_velocity.z),
        QVector3D(msg->linear_acceleration.x,
                  msg->linear_acceleration.y,
                  msg->linear_acceleration.z));
}

/******************************************************************************
** Function:    setTopic
** Author:      Matt Richard
** Parameters:  std::string topic - 
** Returns:     void
** Description:
******************************************************************************/
void ImuNode::setTopic(const std::string &topic)
{
	topic_name = topic;
}

/******************************************************************************
** Function:    getTopic
** Author:      Matt Richard
** Parameters:  None
** Returns:     std::string
** Description: 
******************************************************************************/
std::string ImuNode::getTopic() const
{
	return topic_name;
}
