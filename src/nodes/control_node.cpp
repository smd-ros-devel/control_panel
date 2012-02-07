/******************************************************************************
 * control_node.cpp
 *
 * Author:      Matt Richard
 * Date:        Aug 31, 2011
 * Description: ROS node for controlling a robot's movement. The node receives
 *              a command (keyboard or joystick) and translates it into linear
 *              and angular x, y, and z velocity.
 *****************************************************************************/
#include "control_panel/nodes/control_node.h"


ControlNode::ControlNode(ros::NodeHandle *nh_ptr)
{
	topic_name = Globals::DEFAULT_CONTROL_TOPIC;

	twist_msg.linear.x = 0.0;
	twist_msg.linear.y = 0.0;
	twist_msg.linear.z = 0.0;
	twist_msg.angular.x = 0.0;
	twist_msg.angular.y = 0.0;
	twist_msg.angular.z = 0.0;

	scale = 0.5;

	nh = nh_ptr;
}

void ControlNode::advertise()
{
	control_pub = nh->advertise<geometry_msgs::Twist>(topic_name, 1);
}

double ControlNode::getAngularX() const
{
	return twist_msg.angular.x;
}

double ControlNode::getAngularY() const
{
	return twist_msg.angular.y;
}

double ControlNode::getAngularZ() const
{
	return twist_msg.angular.z;
}

double ControlNode::getLinearX() const
{
	return twist_msg.linear.x;
}

double ControlNode::getLinearY() const
{
	return twist_msg.linear.y;
}

double ControlNode::getLinearZ() const
{
	return twist_msg.linear.z;
}

double ControlNode::getScale() const
{
	return scale;
}

std::string ControlNode::getTopic() const
{
    return topic_name;
}

geometry_msgs::Twist ControlNode::getTwist() const
{
	return twist_msg;
}

void ControlNode::publish()
{
	control_pub.publish(twist_msg);
}

void ControlNode::setAngularX(double x)
{
    if(validVelocity(x))
        twist_msg.angular.x = x * scale;
}

void ControlNode::setAngularY(double y)
{
    if(validVelocity(y))
        twist_msg.angular.y = y * scale;
}

void ControlNode::setAngularZ(double z)
{
    if(validVelocity(z))
        twist_msg.angular.z = z * scale;
}

void ControlNode::setLinearX(double x)
{
	if(validVelocity(x))
		twist_msg.linear.x = x * scale;
}

void ControlNode::setLinearY(double y)
{
	if(validVelocity(y))
		twist_msg.linear.y = y * scale;
}

void ControlNode::setLinearZ(double z)
{
	if(validVelocity(z))
		twist_msg.linear.z = z * scale;
}

void ControlNode::setScale(double s)
{
    if(s > 1.0)
        scale = 1.0;
    else if(s < 0.0)
        scale = 0.0;
    else
        scale = s;
//	if(s >= 0.0 && s <= 1.0)
//		scale = s;
//	else
//		ROS_WARN("Invalid scale: %f", s);
}

void ControlNode::setTopic(const std::string &topic)
{
    topic_name = topic;
}

void ControlNode::unadvertise()
{
    control_pub.shutdown();
}

/******************************************************************************
 * Public Slots
 *****************************************************************************/

void ControlNode::setAngularVector(double x, double y, double z)
{
	setAngularX(x);
	setAngularY(y);
	setAngularZ(z);
}

void ControlNode::setLinearVector(double x, double y, double z)
{
	setLinearX(x);
	setLinearY(y);
	setLinearZ(z);
}

void ControlNode::setTwist(const geometry_msgs::Twist &twist)
{
	twist_msg = twist;
}

void ControlNode::setTwist(double lx, double ly, double lz,double ax, 
	double ay, double az)
{
    setLinearX(lx);
    setLinearY(ly);
    setLinearZ(lz);
    setAngularX(ax);
    setAngularY(ay);
    setAngularZ(az);
}

/******************************************************************************
 *                           Private Functions
 *****************************************************************************/

bool ControlNode::validVelocity(double value)
{
	if(value >= -1.0 && value <= 1.0)
		return true;

	ROS_WARN("Invalid velocity: %f", value);

	return false;
}
