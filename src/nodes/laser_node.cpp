/******************************************************************************
** laser_node.cpp
**
** Author:      Matt Richard
** Date:        Nov. 16, 2011
** Description: ROS node for receiving a ROS LaserScan.msg message.
******************************************************************************/

#include "control_panel/nodes/laser_node.h"

/******************************************************************************
** Function:    LaserNode
** Author:      Matt Richard
** Parameters:  ros::NodeHandle *nh_ptr - 
** Returns:     None
** Description: Constructor.
******************************************************************************/
LaserNode::LaserNode(ros::NodeHandle *nh_ptr)
{
	topic_name = Globals::DEFAULT_LASER_TOPIC;

	nh = nh_ptr;

    white = qRgb(255, 255, 255);
    red = qRgb(255, 0, 0);
}

/******************************************************************************
** Function:    subscribe
** Author:      Matt Richard
** Parameters:  None
** Returns:     void
** Description:
******************************************************************************/
void LaserNode::subscribe()
{
    laser_sub = nh->subscribe(topic_name, 1, &LaserNode::laserCallback, this);
}

/******************************************************************************
** Function:    unsubscribe
** Author:      Matt Richard
** Parameters:  None
** Returns:     void
** Description:
******************************************************************************/
void LaserNode::unsubscribe()
{
    laser_sub.shutdown();
}

/******************************************************************************
** Function:    laserCallback
** Author:      Matt Richard
** Parameters:  const sensor_msgs::LaserScanConstPtr &msg - 
** Returns:     void
** Description:
******************************************************************************/
void LaserNode::laserCallback(const sensor_msgs::LaserScanConstPtr &msg)
{
    int scale = 40; // scale size of image
    double x, y;

    int robot_pos = msg->range_max * scale;
    int height = robot_pos * (1.0 - cos(msg->angle_max));

    QImage buffer(2.0 * robot_pos, height, QImage::Format_RGB888);
    buffer.fill(Qt::black);

    double curr_angle = msg->angle_min + Globals::PI_OVER_TWO;
    // Convert from polar to x,y coordinates and create the image
    for(unsigned int i = 0; i < msg->ranges.size(); i++)
    {
        // Make sure the range is valid
        if(msg->ranges[i] > msg->range_min && msg->ranges[i] < (msg->range_max - 0.1))
        {
            /* x = r * cos(theta) */
            x = msg->ranges[i] * cos(curr_angle);
            /* y = r * sin(theta) */
            y = msg->ranges[i] * sin(curr_angle);

            // draw the laser value on the image
            buffer.setPixel(x * scale + (robot_pos - 1), (robot_pos - 1) - y * scale, white);
        }
        curr_angle += msg->angle_increment;
    }


    // Draw the robots position on the image
    for(int i = robot_pos - 1; i > robot_pos - 6; i--)
        for(int j = robot_pos - 3; j < robot_pos + 3; j++)
            buffer.setPixel(j, i, red);

    emit laserScanReceived(buffer, scale);
}

/******************************************************************************
** Function:    setTopic
** Author:      Matt Richard
** Parameters:  std::string topic -
** Returns:     void
** Description:
******************************************************************************/
void LaserNode::setTopic(const std::string &topic)
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
std::string LaserNode::getTopic() const
{
	return topic_name;
}
