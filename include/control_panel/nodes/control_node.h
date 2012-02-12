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
 * \file   control_node.h
 * \date   Aug 31, 2011
 * \author Matt Richard
 */
#ifndef CONTROL_PANEL_CONTROL_NODE_H
#define CONTROL_PANEL_CONTROL_NODE_H

#include <QObject>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <stdio.h>
#include <string>
#include "control_panel/globals.h"

/**
 * \class ControlNode
 * \brief ROS node for controlling a robot's movement
 */
class ControlNode : public QObject
{
	Q_OBJECT

	public:
		ControlNode(ros::NodeHandle *nh_ptr);
		void advertise();
		double getAngularX() const;
		double getAngularY() const;
		double getAngularZ() const;
		double getLinearX() const;
		double getLinearY() const;
		double getLinearZ() const;
		double getScale() const;
        std::string getTopic() const;
		geometry_msgs::Twist getTwist() const;
		//void publish();
        void setAngularX(double x);
        void setAngularY(double y);
        void setAngularZ(double z);
        void setLinearX(double x);
        void setLinearY(double y);
        void setLinearZ(double z);
		void setScale(double s);
		void setTopic(const std::string &topic);
        void unadvertise();

	public slots:
        void publish();
        void setAngularVector(double x = 0.0, double y = 0.0, double z = 0.0);
        void setLinearVector(double x = 0.0, double y = 0.0, double z = 0.0);
		void setTwist(const geometry_msgs::Twist &twist);
        void setTwist(double lx = 0.0, double ly = 0.0, double lz = 0.0, 
                      double ax = 0.0, double ay = 0.0, double az = 0.0);

	private:
		bool validVelocity(double value);

		std::string topic_name;
		ros::NodeHandle *nh;
		ros::Publisher control_pub;
		geometry_msgs::Twist twist_msg;
		double scale;
};

#endif // CONTROL_PANEL_CONTROL_NODE_H
