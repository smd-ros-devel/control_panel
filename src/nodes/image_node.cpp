/* @todo Add license here */

/**
 * \file   image_node.cpp
 * \date   Sept 8, 2011
 * \author Matt Richard
 */
#include "control_panel/nodes/image_node.h"

ImageNode::ImageNode(ros::NodeHandle *nh_ptr)
{
	topic_name = Globals::DEFAULT_CAMERA_TOPIC;

	nh = nh_ptr;

	it = new image_transport::ImageTransport(*nh);
}

ImageNode::~ImageNode()
{
	unsubscribe();

	delete it;
}

void ImageNode::subscribe()
{
	image_sub = it->subscribe(topic_name, 1, &ImageNode::imageCallback, this);//,
		//image_transport::TransportHints("raw", ros::TransportHints().unreliable()));
}

void ImageNode::unsubscribe()
{
	image_sub.shutdown();
}

void ImageNode::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
	cv_bridge::CvImageConstPtr cv_ptr;

	// Convert the ROS image into a CvMat image
	if(enc::isColor(msg->encoding) || enc::isMono(msg->encoding))
	{
		cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);

		QImage buffer((unsigned char *)cv_ptr->image.data, cv_ptr->image.cols,
			cv_ptr->image.rows, QImage::Format_RGB888);

        emit frameReceived(buffer.rgbSwapped());
    }
	else if(msg->encoding == enc::TYPE_32FC1)
	{
		cv_ptr = cv_bridge::toCvShare(msg, enc::TYPE_32FC1);

		cv::Mat depth_img;// = cv::Mat(cv_ptr->image.size(), CV_8UC3, cv::Scalar(0));//, 0, 255));
		cv::Mat temp_img;// = cv::Mat(cv_ptr->image.size(), CV_8UC1);

		cv_ptr->image.convertTo(temp_img, CV_8UC1, 40.0);//255.0/2048.0);
		cv::cvtColor(temp_img, depth_img, CV_GRAY2BGR);

		QImage buffer((unsigned char *)depth_img.data, depth_img.cols,
			depth_img.rows, QImage::Format_RGB888);

        emit frameReceived(buffer.rgbSwapped());
	}
	else
		ROS_WARN("Unrecognized image encoding: %s\n", msg->encoding.c_str());
}
