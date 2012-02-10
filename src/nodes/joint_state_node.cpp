/* @todo Add license here */

/**
 * \file   joint_state_node.cpp
 * \date   Dec 7, 2011
 * \author Matt Richard
 */
#include "control_panel/nodes/joint_state_node.h"

JointStateNode::JointStateNode(ros::NodeHandle *nh_ptr)
{
	topic_name = Globals::DEFAULT_JOINT_TOPIC;

	nh = nh_ptr;
}

void JointStateNode::subscribe()
{
	joint_sub = nh->subscribe(topic_name, 1, &JointStateNode::jointCallback, this);
}

void JointStateNode::unsubscribe()
{
	joint_sub.shutdown();
}

void JointStateNode::jointCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    QStringList name_list;

    // Store all joint names in a QStringList
    for(unsigned int i = 0; i < msg->name.size(); i++)
        name_list << msg->name[i].c_str();

    emit jointDataReceived(name_list, msg->position, msg->velocity, msg->effort);
}
