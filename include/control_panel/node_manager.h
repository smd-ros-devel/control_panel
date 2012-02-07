/******************************************************************************
 * node_manager.h
 *
 * Author:      Matt Richard, Scott Logan
 * Date:        Aug. 4, 2011
 * Description: Handles all ROS nodes for a single robot.
 *****************************************************************************/

#ifndef CONTROL_PANEL_NODE_MANAGER_H
#define CONTROL_PANEL_NODE_MANAGER_H

#include <QThread>
#include <QTimer>
#include <QList>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include <string>
#include <stdio.h>
#include "nodes/control_node.h"
#include "nodes/command_node.h"
#include "nodes/diagnostic_node.h"
#include "nodes/gps_node.h"
#include "nodes/image_node.h"
#include "nodes/imu_node.h"
#include "nodes/joint_node.h"
#include "nodes/laser_node.h"
#include "nodes/map_node.h"
#include "nodes/odometry_node.h"
#include "nodes/range_node.h"
#include "globals.h"
#include "robot_config.h"


class NodeManager : public QThread
{
	Q_OBJECT

	public:
		NodeManager(struct RobotConfig *);
		~NodeManager();
		void run();
		void stop();
		bool isConnected() const;
        //DiagnosticNode *addDiagnosticNode();
        GpsNode *addGpsNode(const std::string &topic);
        ImuNode *addImuNode(const std::string &topic);
        OdometryNode *addOdometryNode(const std::string &topic);
        //JointNode *addJointNode();
        //OdometryNode *addOdometryNode();
        //RangeNode *addRangeNode();

		// ROS Nodes
		ImageNode *camera_node;
		ControlNode *control_node;
		CommandNode *command_node;
		DiagnosticNode *diagnostic_node;
		GpsNode *gps_node;
		ImuNode *imu_node;
		JointNode *joint_node;
		LaserNode *laser_node;
		MapNode *map_node;
		OdometryNode *odometry_node;
        RangeNode *range_node;

	public slots:
		void changeRawDataSource(const std::string &source);
        //void changeProcessedDataSource(const std::string &source);

	signals:
		void connectionStatusChanged(int status);

	private:
		ros::NodeHandle *nh_ptr;
		ros::CallbackQueue robot_callback_queue;
		struct RobotConfig *robot_config;
		bool connected;
        QTimer *pub_timer;

        QList<ImuNode *> *imu_node_list;
        QList<GpsNode *> *gps_node_list;
        QList<OdometryNode *> *odom_node_list;
};

#endif // CONTROL_PANEL_NODE_MANAGER_H
