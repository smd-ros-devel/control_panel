/* @todo Add license here */

/**
 * \file   disparity_node.cpp
 * \date   Feb 5, 2012
 * \author Matt Richard
 */
#include "control_panel/nodes/disparity_image_node.h"
//#include "sensor_msgs/Image.h"

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

void DisparityImageNode::disparityCallback(const stereo_msgs::DisparityImageConstPtr &msg)
{
    /* @todo Convert msg to a QImage and emit */
    cv_bridge::CvImageConstPtr cv_ptr;

    if(msg->image.encoding != enc::TYPE_32FC1)
    {
        ROS_ERROR("Unusable disparity image encoding '%s'", msg->image.encoding.c_str());
        return;
    }

    const cv::Mat_<float> dmat(msg->image.height, msg->image.width, (float *)&msg->image.data[0], msg->image.step);
    // cv_ptr = cv_bridge::toCvCopy(msg->image, enc::BGR8);
    /* @todo Finish conversion from disparity image to QImage */
}
