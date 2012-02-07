/**
 * range_node.cpp
 *
 * Author:
 * Date:
 * Description: This node receives Range.msg which is used for 1-D Infared
 *              sensors or unltrasound/sonar sensors.
 **/

#include "control_panel/nodes/range_node.h"

RangeNode::RangeNode(ros::NodeHandle *nh_ptr)
{
    topic_name = Globals::DEFAULT_RANGE_TOPIC;

    nh = nh_ptr;
}

void RangeNode::subscribe()
{
    range_sub = nh->subscribe(topic_name, 1, &RangeNode::rangeCallback, this);
}

std::string RangeNode::getTopic() const
{
    return topic_name;
}

void RangeNode::rangeCallback(const sensor_msgs::RangeConstPtr &msg)
{
    bool valid = true;

    if(msg->range < msg->min_range || msg->range > msg->max_range)
        valid = false;

    emit rangeReceived(msg->range, valid);
}

void RangeNode::setTopic(const std::string &topic)
{
    topic_name = topic;
}

void RangeNode::unsubscribe()
{
    range_sub.shutdown();
}
