/**
 * range_node.h
 *
 * Author:      Matt Richard
 * Date:        Jan 18, 2012
 * Description: This node receives Range.msg which is used for 1-D Infared
 *              sensors or unltrasound/sonar sensors.
 **/

#ifndef CONTROL_PANEL_RANGE_NODE_H
#define CONTROL_PANEL_RANGE_NODE_H

#include <QObject>
#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include <string>
#include "control_panel/globals.h"


class RangeNode : public QObject
{
    Q_OBJECT

    public:
        RangeNode(ros::NodeHandle *nh_ptr);
        void subscribe();
        std::string getTopic() const;
        void rangeCallback(const sensor_msgs::RangeConstPtr &msg);
        void setTopic(const std::string &topic);
        void unsubscribe();

    signals:
        void rangeReceived(float value, bool in_range);

    private:
        ros::NodeHandle *nh;
        ros::Subscriber range_sub;
        std::string topic_name;
};

#endif // CONTROL_PANEL_RANGE_NODE_H
