/* @todo Add license here */

/**
 * \file   node_manager.cpp
 * \date   Aug 4, 2011
 * \author Matt Richard, Scott K Logan
 */
#include "control_panel/node_manager.h"
#include <stdio.h>

NodeManager::NodeManager(struct RobotConfig *new_robot_config) :
	camera_node(NULL), control_node(NULL), command_node(NULL),
	diagnostic_node(NULL), gps_node(NULL), imu_node(NULL), joint_node(NULL),
	laser_node(NULL), map_node(NULL), odometry_node(NULL), range_node(NULL)
{
	connected = false;
	robot_config = new_robot_config;

    gps_node_list = new QList<GpsNode *>;
    imu_node_list = new QList<ImuNode *>;
    odom_node_list = new QList<OdometryNode *>;

    // Create the node handle and set the local callback queue
	nh_ptr = new ros::NodeHandle(robot_config->nameSpace.toStdString());
	nh_ptr->setCallbackQueue(&robot_callback_queue);

	// Inititalize ROS nodes
	if(robot_config->sensors.cameras)
		camera_node = new ImageNode(nh_ptr);
	if(robot_config->controls.used)
    {
		control_node = new ControlNode(nh_ptr);
        control_node->setTopic(robot_config->controls.drive->topicName.toStdString());

        pub_timer = new QTimer(this);
        connect(pub_timer, SIGNAL(timeout()), control_node, SLOT(publish()));

        joystick_node = new JoystickNode(nh_ptr);
        connect(joystick_node, SIGNAL(axis_event(int, double)),
            this, SLOT(joystickAxisChanged(int, double)));
        connect(joystick_node, SIGNAL(button_event(int, bool)),
            this, SLOT(joystickButtonChanged(int, bool)));
    }
    if(robot_config->commands.used)
		command_node = new CommandNode(nh_ptr);
	if(robot_config->diagnostics.used)
		diagnostic_node = new DiagnosticNode(nh_ptr);
	if(false)
		joint_node = new JointNode(nh_ptr);
	if(robot_config->sensors.lasers)
		laser_node = new LaserNode(nh_ptr);
	if(robot_config->sensors.maps)
		map_node = new MapNode(nh_ptr);
    if(robot_config->sensors.range)
        range_node = new RangeNode(nh_ptr);
}

/******************************************************************************
 * Function:    run
 * Author:      Matt Richard, Scott Logan
 * Parameters:  None
 * Returns:     void
 * Description: The thread.
 *****************************************************************************/
void NodeManager::run()
{
    int i;

    emit connectionStatusChanged(Globals::Connecting);

	// Start necessary ROS nodes
	if(control_node)
    {
		control_node->advertise();
        pub_timer->start(30);
    }
	if(diagnostic_node)
	{
		diagnostic_node->setTopic(robot_config->diagnostics.topicName.toStdString());
		diagnostic_node->subscribe();
	}
	//if(gps_node)
    for(i = 0; i < gps_node_list->count(); i++)
		((GpsNode *)gps_node_list->at(i))->subscribe();
	//if(imu_node)
    for(i = 0; i < imu_node_list->count(); i++)
		((ImuNode *)imu_node_list->at(i))->subscribe();
    for(i = 0; i < odom_node_list->count(); i++)
        ((OdometryNode *)odom_node_list->at(i))->subscribe();
	if(joint_node)
		joint_node->subscribe();
	if(map_node)
		map_node->subscribe();
	if(odometry_node)
		odometry_node->subscribe();
    if(range_node)
        range_node->subscribe();

	connected = true;
	emit connectionStatusChanged(Globals::Connected);

	printf("Connected to %s\n", robot_config->getRobotName().toStdString().c_str());

	// Loop until disconnected
	while(connected)
		robot_callback_queue.callAvailable(ros::WallDuration(0.01));

	printf("Disconnected from %s\n", robot_config->getRobotName().toStdString().c_str());
}

/******************************************************************************
 * Function:    stop
 * Author:      Matt Richard, Scott Logan
 * Parameters:  None
 * Returns:     void
 * Description: Stops the connection thread.
 *****************************************************************************/
void NodeManager::stop()
{
	if(connected)
	{
		connected = false;

        if(control_node)
            pub_timer->stop();

		printf("Shutting down %s's node handle\n",
			robot_config->getRobotName().toStdString().c_str());
		// Shutdown node handle
		// Why do we need to unsubscribe camera_node here?
		if(camera_node)
			camera_node->unsubscribe();
		nh_ptr->shutdown();

		// Kill thread
		exit();
		wait();

		// Verify the thread was shutdown
		if(isRunning())
			printf("ERROR -- %s's thread failed to stop\n",
				robot_config->robotName.toStdString().c_str());

		emit connectionStatusChanged(Globals::Disconnected);
	}
}

/******************************************************************************
 * Function:    isConnected
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     bool - true if the robot is connected, otherwise false.
 * Description: Returns the connection state of the Control Panel with the
 *              robot.
 *****************************************************************************/
bool NodeManager::isConnected() const
{
	return connected;
}

/******************************************************************************
 * Function:    changeRawDataSource
 * Author:      Scott Logan
 * Parameters:  const std::string &source
 * Returns:     void
 * Description:
 *****************************************************************************/
void NodeManager::changeRawDataSource(const std::string &source)
{
	if(camera_node)
		camera_node->unsubscribe();
	if(laser_node)
		laser_node->unsubscribe();

	RobotCamera *cam = robot_config->sensors.cameras;
	while(cam != NULL)
	{
		if(source == cam->name.toStdString())
		{
			camera_node->setTopic(cam->topicName.toStdString());
			camera_node->subscribe();
			return;
		}
		else
			cam = cam->next;
	}

	RobotLaser *laser = robot_config->sensors.lasers;
	while(laser != NULL)
	{
		if(source == laser->name.toStdString())
		{
			laser_node->setTopic(laser->topicName.toStdString());
			laser_node->subscribe();
			return;
		}
		else
			laser = laser->next;
	}
}



GpsNode *NodeManager::addGpsNode(const std::string &topic)
{
    gps_node = new GpsNode(nh_ptr);
    gps_node->setTopic(topic);

    gps_node_list->append(gps_node);

    return gps_node;
}

ImuNode *NodeManager::addImuNode(const std::string &topic)
{
    imu_node = new ImuNode(nh_ptr);
    imu_node->setTopic(topic);

    imu_node_list->append(imu_node);

    return imu_node;
}


OdometryNode *NodeManager::addOdometryNode(const std::string &topic)
{
    odometry_node = new OdometryNode(nh_ptr);
    odometry_node->setTopic(topic);

    odom_node_list->append(odometry_node);

    return odometry_node;
}

/*
RangeNode *NodeManager::addRangeNode()
{
    range_node = new RangeNode(nh_ptr);

    // @todo
    // Store pointer.

    return range_node;
}
*/


void NodeManager::joystickAxisChanged(int axis, double value)
{
	if(connected)
	{
		// Joystick mapping here
		if(axis == 0 && control_node)
			control_node->setLinearY(value);
		else if(axis == 1 && control_node)
			control_node->setLinearX(value);
		else if(axis == 2 && control_node)
			control_node->setAngularZ(value);
		else if(axis == 3 && control_node)
			control_node->setLinearZ(value);
	}
}

void NodeManager::joystickButtonChanged(int button, bool state)
{
	if(connected && state)
	{
		// Joystick mapping here
		if(button == 14)
		{
			// Takeoff Message
			struct RobotCommandCustom *custom_temp = robot_config->commands.custom;
			while(custom_temp != NULL)
			{
				if(custom_temp->name == "takeoff")
				    command_node->callEmpty(custom_temp->topicName);
				custom_temp = custom_temp->next;
			}
		}
		if(button == 13)
		{
			// Land Message
			struct RobotCommandCustom *custom_temp = robot_config->commands.custom;
			while(custom_temp != NULL)
			{
				if(custom_temp->name == "land")
					command_node->callEmpty(custom_temp->topicName);
				custom_temp = custom_temp->next;
			}
		}
		if(button == 12)
		{
			// Reset Message
			struct RobotCommandCustom *custom_temp = robot_config->commands.custom;
			while(custom_temp != NULL)
			{
				if(custom_temp->name == "Reset")
					command_node->callEmpty(custom_temp->topicName);
				custom_temp = custom_temp->next;
			}
		}
		if(button == 15)
		{
			// Land Message
			struct RobotCommandCustom *custom_temp = robot_config->commands.custom;
			while(custom_temp != NULL)
			{
				if(custom_temp->name == "Camera Toggle")
					command_node->callEmpty(custom_temp->topicName);
				custom_temp = custom_temp->next;
			}
		}
	}
}
