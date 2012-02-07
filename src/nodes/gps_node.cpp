/******************************************************************************
** gps_node.cpp
**
** Author:      Matt Richard
** Date:        Aug 31, 2011
** Description: ROS node for received gps data.
******************************************************************************/

#include "control_panel/nodes/gps_node.h"

/******************************************************************************
** Function:    GpsNode
** Author:      Matt Richard
** Parameters:  ros::NodeHandle *nh_ptr - 
** Returns:     None
** Description: Constructor.
******************************************************************************/
GpsNode::GpsNode(ros::NodeHandle *nh_ptr)
{
	topic_name = Globals::DEFAULT_GPS_TOPIC;

	nh = nh_ptr;
}

/******************************************************************************
** Function:    subscribe
** Author:      Matt Richard
** Parameters:  None
** Returns:     void
** Description: 
******************************************************************************/
void GpsNode::subscribe()
{
	gps_sub = nh->subscribe(topic_name, 1, &GpsNode::gpsCallback, this);
}

/******************************************************************************
** Function:    unsubscribe
** Author:      Matt Richard
** Parameters:  None
** Returns:     void
** Description:
******************************************************************************/
void GpsNode::unsubscribe()
{
	gps_sub.shutdown();
}

/******************************************************************************
** Function:    gpsCallback
** Author:      Matt Richard
** Parameters:  const sensor_msgs::NavSatFixConstPtr &msg -
** Returns:     void
** Description:
******************************************************************************/
void GpsNode::gpsCallback(const sensor_msgs::NavSatFixConstPtr &msg)
{
	emit gpsDataReceived(msg->latitude, msg->longitude, msg->altitude);
}

/******************************************************************************
** Function:    setTopic
** Author:      Matt Richard
** Parameters:  std::string topic -
** Returns:     void
** Description:
******************************************************************************/
void GpsNode::setTopic(const std::string &topic)
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
std::string GpsNode::getTopic() const
{
	return topic_name;
}
