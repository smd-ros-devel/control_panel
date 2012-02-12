/*
 * Copyright (c) 2011, 2012 SDSM&T RIAS.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


/**
 * \file   control_node.cpp
 * \date   Aug 31, 2011
 * \author Matt Richard
 * \brief  ROS node for controlling a robot's movement.
 */
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
