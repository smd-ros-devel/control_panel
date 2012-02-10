
/**
 * \file   disparity_node.cpp
 * \date   Feb 5, 2012
 * \author Matt Richard
 */
#include "control_panel/nodes/disparity_node.h"

DisparityNode::DisparityNode(ros::NodeHandle *nh_ptr)
{
    topic_name = "stereo/disparity";

    nh = nh_ptr;
}

void DisparityNode::subscribe()
{
    disparity_sub = nh->subscribe<stereo_msgs::DisparityImage>(topic_name,
        1, &DisparityNode::disparityCallback, this);
}

void DisparityNode::unsubscribe()
{
    disparity_sub.shutdown();
}

void DisparityNode::disparityCallback(const stereo_msgs::DisparityImage::ConstPtr &msg)
{
    /* @todo Convert msg to a QImage and emit */
}
