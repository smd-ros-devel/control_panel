/**
 * mav_node.cpp
 *
 * Author:      Matt Richard
 * Date:        Jan 15, 2011
 * Description:
 **/

#include "control_panel/nodes/mav_node.h"

MavNode::MavNode(ros::NodeHandle *nh_ptr)
{
    nh = nh_ptr;
}

/*
void MavNode::callChangeDesPose(int field, double delta, const QString &topic)
{
    mav_msgs::ChangeDesPose srv;
    srv.request.field = field;
    srv.request.delta = delta;

    ros::ServiceClient client = nh->serviceClient<mav_msgs::ChangeDesPose>(topic);

    if(!client.call(srv))
        ROS_INFO("Failed to call ChangeDesPose service\n");
}
*/

/*
bool MavNode::callGetMotorsOnOff(const QString &topic)
{
    mav_msgs::GetMotorsOnOff srv;

    ros::ServiceClient client = nh->serviceClient<mav_msgs::GetMotorsOnOff>(topic);

    if(client.call(srv))
        return srv.response.on;
    
    ROS_INFO("Failed to call GetMotorsOnOff service\n");
    return false;
}
*/

void MavNode::callLand(const std::string &topic)
{
    mav_msgs::Land srv;
    
    ros::ServiceClient client = nh->serviceClient<mav_msgs::Land>(topic);

    if(!client.call(srv))
        ROS_INFO("Failed to call Land service");
}

/*
void MavNode::callSetMotorsOnOff(bool state, const QString &topic)
{
    mav_msgs::SetMotorsOnOff srv;
    srv.request.on = state;

    ros::ServiceClient client = nh->serviceClient<mav_msgs::SetMotorsOnOff>(topic);

    if(!client.call(srv))
        ROS_INFO("Failed to call SetMotorsOnOff service\n");
}
*/

void MavNode::callTakeoff(const std::string &topic)
{
    mav_msgs::Takeoff srv;

    ros::ServiceClient client = nh->serviceClient<mav_msgs::Takeoff>(topic);

    if(!client.call(srv))
        ROS_INFO("Failed to call Takeoff service");
}

/*
void MavNode::serviceToggleEngage()
{
    toggle_engage_client = nh->serviceClient<mav_msgs::ToggleEngage>(
        toggle_engage_topic);
}
*/
