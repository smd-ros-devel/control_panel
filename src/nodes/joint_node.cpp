/* @todo Add license here */

/**
 * \file   joint_node.cpp
 * \date   Dec 7, 2011
 * \author Matt Richard
 */
#include "control_panel/nodes/joint_node.h"

JointNode::JointNode(ros::NodeHandle *nh_ptr)
{
	topic_name = Globals::DEFAULT_JOINT_TOPIC;

	nh = nh_ptr;
}

void JointNode::subscribe()
{
	joint_sub = nh->subscribe(topic_name, 1, &JointNode::jointCallback, this);
}

void JointNode::unsubscribe()
{
	joint_sub.shutdown();
}

void JointNode::jointCallback(const sensor_msgs::JointStateConstPtr &msg)
{
    /* @todo Finish this function */
    QStringList name_list;

    for(unsigned int i = 0; i < msg->name.size(); i++)
    {
        name_list << msg->name[i].c_str();
    }

    emit jointDataReceived(name_list, msg->position, msg->velocity, msg->effort);
}
