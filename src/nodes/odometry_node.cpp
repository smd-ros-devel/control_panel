/******************************************************************************
 * odometry_node.cpp
 *
 * Author:      Matt Richard
 * Date:        Sept 8, 2011
 * Description: This receives a robot's estimated odomentry (estimated position
 *              and velocity).
 *****************************************************************************/

#include "control_panel/nodes/odometry_node.h"

/******************************************************************************
 * Function:    OdometryNode
 * Author:      Matt Richard
 * Parameters:  ros::NodeHandle *nh_ptr - 
 * Returns:     None
 * Description: Constructor.
 *****************************************************************************/
OdometryNode::OdometryNode(ros::NodeHandle *nh_ptr)
{
	topic_name = Globals::DEFAULT_ODOMETRY_TOPIC;

	nh = nh_ptr;
}

/******************************************************************************
 * Function:    subscribe
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     void
 * Description: 
 *****************************************************************************/
void OdometryNode::subscribe()
{
	odometry_sub = nh->subscribe(topic_name, 1,
		&OdometryNode::odometryCallback, this);
}

/******************************************************************************
 * Function:    unsubscribe
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     void
 * Description:
 *****************************************************************************/
void OdometryNode::unsubscribe()
{
	odometry_sub.shutdown();
}

/******************************************************************************
 * Function:    odometryCallback
 * Author:      Matt Richard
 * Parameters:  const nav_msgs::OdometryConstPtr &msg - 
 * Returns:     void
 * Description:
 *****************************************************************************/
void OdometryNode::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
{
    emit odometryDataReceived(
        QVector3D(msg->pose.pose.position.x,
                  msg->pose.pose.position.y,
                  msg->pose.pose.position.z),
        QQuaternion(msg->pose.pose.orientation.w,
                    msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z),
        QVector3D(msg->twist.twist.linear.x,
                  msg->twist.twist.linear.y,
                  msg->twist.twist.linear.z),
        QVector3D(msg->twist.twist.angular.x,
                  msg->twist.twist.angular.y,
                  msg->twist.twist.angular.z));
}

/******************************************************************************
 * Function:    setTopic
 * Author:      Matt Richard
 * Parameters:  std::string topic - 
 * Returns:     void
 * Description:
 *****************************************************************************/
void OdometryNode::setTopic(const std::string &topic)
{
	topic_name = topic;
}

/******************************************************************************
 * Function:    getTopic
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     std::string - 
 * Description:
 *****************************************************************************/
std::string OdometryNode::getTopic() const
{
	return topic_name;
}
