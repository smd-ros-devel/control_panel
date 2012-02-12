/*
 * Copyright (c) 2011, 2012 SDSM&T RIAS.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


/**
 * \file   grid_cells_node.h
 * \date   Jan 27, 2012
 * \author Matt Richard
 */
#ifndef CONTROL_PANEL_GRID_CELLS_NODE_H
#define CONTROL_PANEL_GRID_CELLS_NODE_H

#include <QObject>
#include "ros/ros.h"
#include "nav_msgs/GridCells.h"
#include <string>


/**
 * \class GridCellsNode
 * \brief ROS node that receives a nav_msgs::GridCells message
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
