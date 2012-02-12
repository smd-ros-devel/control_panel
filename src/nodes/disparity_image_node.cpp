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

void DisparityImageNode::disparityCallback(const stereo_msgs::DisparityImageConstPtr &msg)
{
    if(msg->image.encoding != enc::TYPE_32FC1)
    {
        ROS_ERROR("Unusable disparity image encoding '%s'", msg->image.encoding.c_str());
        return;
    }

    float mult = 255.0 / (msg->max_disparity - msg->min_disparity);

    const cv::Mat_<float> d_mat(msg->image.height, msg->image.width,
        (float *)&msg->image.data[0], msg->image.step);

    cv::Mat_<cv::Vec3b> disp_color;
    disp_color.create(msg->image.height, msg->image.width);

    // Create the disparity color image
    for(int i = 0; i < disp_color.rows; i++)
    {
        const float *d = d_mat[i];

        for(int j = 0; j < disp_color.cols; j++)
        {
            int index = (d[j] - msg->min_disparity) * mult + 0.5;
            index = std::min(255, std::max(0, index));

            disp_color(i, j)[2] = color_table[3 * index];
            disp_color(i, j)[1] = color_table[1 + (3 * index)];
            disp_color(i, j)[0] = color_table[2 + (3 * index)];
        }
    }

    // Create the QImage from the disparity color image
    QImage buffer((unsigned char *)disp_color.data, disp_color.cols,
        disp_color.rows, QImage::Format_RGB888);

    emit disparityImageReceived(buffer.rgbSwapped());
}
