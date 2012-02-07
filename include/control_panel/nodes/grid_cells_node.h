/**
 * grid_cells_node.h
 *
 * Author:      Matt Richard
 * Date:        Jan 27, 2012
 * Description: Node to receive an array of cells in a 2D grid (GridCells.msg).
 */

#ifndef CONTROL_PANEL_GRID_CELLS_NODE_H
#define CONTROL_PANEL_GRID_CELLS_NODE_H

#include <QObject>
#include "ros/ros.h"
#include "nav_msgs/GridCells.h"
#include <string>


/**
 * \class  GridCellsNode
 * \author Matt Richard
 * \brief
 */
class GridCellsNode : public QObject
{
    Q_OBJECT

    public:
        //GridCellsNode(ros::NodeHandle *nh_ptr);
        //std::string getTopic() const;
        //void setTopic(const std::string &topic);
        //void subscribe();
        //void unsubscribe();

    protected:
        //void gridCellsCallback(const nav_msgs::GridCellsConstPtr &msg);

    signals:
        //void gridCellsDataReceived( ??? );

    private:
        ros::NodeHandle *nh;
        ros::Subscriber grid_cells_sub;
        std::string topic_name;
};

#endif // CONTROL_PANEL_GRID_CELLS_NODE_H
