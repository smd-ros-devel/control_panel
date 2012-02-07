/**
 * mav_node.h
 *
 * Author:      Matt Richard
 * Date:        Jan 13, 2011
 * Description:
 **/

#ifndef CONTROL_PANEL_MAV_NODE_H
#define CONTROL_PANEL_MAV_NODE_H

#include <QObject>
#include "ros/ros.h"
//#include "mav_msgs/Height.h"
//#include "mav_msgs/State.h"
//#include "mav_msgs/ChangeDesPose.h"
//#include "mav_msgs/GetMotorsOnOff.h"
#include "mav_msgs/Land.h"
//#include "mav_msgs/SetMotorsOnOff.h"
#include "mav_msgs/Takeoff.h"
//#include "mav_msgs/ToggleEngage.h"
#include <string>


class MavNode : public QObject
{
    Q_OBJECT

    public:
        MavNode(ros::NodeHandle *nh_ptr);

    public slots:
        //void callChangeDesPose(short int field, double delta);
        //void callGetMotorsOnOff();
        void callLand(const std::string &topic);
        //void callSetMotorsOnOff(bool state);
        void callTakeoff(const std::string &topic);
        //void callToggleEngage();
/*
    public slots:
        void changeDesPose(short int field, double delta);
        void getMotorsOnOff();
        void land();
        void setMotorsOnOff(bool state);
        void takeoff();
        void toggleEngage();
*/
//    signals:
        

    private:
/*
        std::string height_topic;
        //std::string state_topic;
        std::string change_des_pose_topic;
        std::string get_motors_topic;
        std::string land_topic;
        std::string set_motors_topic;
        std::string takeoff_topic;
        std::string toggle_engage_topic;
*/
        ros::NodeHandle *nh;
/*
        ros::Subscriber height_sub;
        //ros::Subscriber state_sub;
        ros::ServiceClient change_des_pose_client;
        ros::ServiceClient get_motors_client;
        ros::ServiceClient land_client;
        ros::ServiceClient set_motors_client;
        ros::ServiceClient takeoff_client;
        ros::ServiceClient toggle_engage_client;

        mav_msgs::Height height_srv;
        //mav_msgs::State state_msg;
        mav_msgs::ChangeDesPose change_des_pose_srv;
        mav_msgs::GetMotorsOnOff get_motors_srv;
        mav_msgs::Land land_srv;
        mav_msgs::SetMotorsOnOff set_motors_srv;
        mav_msgs::Takeoff takeoff_srv;
        mav_msgs::ToggleEngage toggle_engage_srv;
*/
};

#endif // CONTROL_PANEL_MAV_NODE_H

