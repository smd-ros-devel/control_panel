/* @todo Add license here */

/**
 * \file   disparity_node.cpp
 * \date   Feb 5, 2012
 * \author Matt Richard
 */
#include "control_panel/nodes/disparity_image_node.h"

DisparityImageNode::DisparityImageNode(ros::NodeHandle *nh_ptr)
{
    topic_name = "stereo/disparity";

    nh = nh_ptr;
}

void DisparityImageNode::subscribe()
{
    disparity_sub = nh->subscribe<stereo_msgs::DisparityImage>(topic_name,
        1, &DisparityImageNode::disparityCallback, this);
}

void DisparityImageNode::unsubscribe()
{
    disparity_sub.shutdown();
}

void DisparityImageNode::disparityCallback(const stereo_msgs::DisparityImage::ConstPtr &msg)
{
    /* @todo Convert msg to a QImage and emit */
}
