/* @todo Add license here */

/**
 * \file   node_manager.h
 * \date   Aug. 4, 2011
 * \author Matt Richard, Scott K Logan
 */
#ifndef CONTROL_PANEL_NODE_MANAGER_H
#define CONTROL_PANEL_NODE_MANAGER_H

#include <QThread>
#include <QTimer>
#include <QList>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include <string>
#include "nodes/control_node.h"
#include "nodes/command_node.h"
#include "nodes/diagnostic_node.h"
#include "nodes/disparity_image_node.h"
#include "nodes/gps_node.h"
#include "nodes/image_node.h"
#include "nodes/imu_node.h"
#include "nodes/joint_state_node.h"
#include "nodes/joystick_node.h"
#include "nodes/laser_node.h"
#include "nodes/map_node.h"
#include "nodes/odometry_node.h"
#include "nodes/range_node.h"
#include "globals.h"
#include "robot_config.h"


/**
 * \class NodeManager
 * \brief Manages all ROS nodes for communicating with a robot
 */
class NodeManager : public QThread
{
	Q_OBJECT

	public:
		NodeManager(struct RobotConfig *);
		void run();
		void stop();
		bool isConnected() const;
        GpsNode *addGpsNode(const std::string &topic = Globals::DEFAULT_GPS_TOPIC);
        ImuNode *addImuNode(const std::string &topic = Globals::DEFAULT_IMU_TOPIC);
        OdometryNode *addOdometryNode(const std::string &topic = Globals::DEFAULT_ODOMETRY_TOPIC);

		// ROS Nodes
		ImageNode *camera_node; // For raw images (e.g., camera/depth image feeds)
        ImageNode *image_node;  // For processed images (e.g., rectified images)
		ControlNode *control_node;
		CommandNode *command_node;
		DiagnosticNode *diagnostic_node;
        DisparityImageNode *disparity_image_node;
		GpsNode *gps_node;
		ImuNode *imu_node;
		JointStateNode *joint_state_node;
        JoystickNode *joystick_node;
		LaserNode *laser_node;
		MapNode *map_node;
		OdometryNode *odometry_node;
        RangeNode *range_node;

	public slots:
		void changeRawDataSource(const std::string &source);
        void changeProcessedDataSource(const std::string &source);
        void joystickAxisChanged(int axis, double value);
        void joystickButtonChanged(int button, bool state);

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
