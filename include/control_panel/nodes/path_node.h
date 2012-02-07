/**
 * path_node.h
 *
 * Author:      Matt Richard
 * Date:        Jan 27, 2012
 * Description: Node to receive the path (an array of poses) of a robot.
 */

#ifndef CONTROL_PANEL_PATH_NODE_H
#define CONTROL_PANEL_PATH_NODE_H

#include <QObject>
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include <string>


/**
 * \class  PathNode
 * \author Matt Richard
 * \brief
 */
class PathNode : public QObject
{
    Q_OBJECT

    public:
        //PathNode(ros::NodeHandle *nh_ptr);
        //std::string getTopic() const;
        //void setTopic(const std::string &topic);
        //void subscribe();
        //void unsubscribe();

    protected:
        //void pathCallback(const nav_msgs::PathConstPtr &msg);

    signals:
        //void pathDataReceived( ??? );

    private:
        ros::NodeHandle *nh;
        ros::Subscriber path_sub;
        std::string topic_name;
};

#endif // CONTROL_PANEL_PATH_NODE_H
