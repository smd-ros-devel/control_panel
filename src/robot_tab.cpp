/**
 * \file   robot_tab.cpp
 * \date   Aug 6, 2011
 * \author Matt Richard, Scott K Logan
 */
#include <QtGui>
#include <QMetaType>
#include "control_panel/robot_tab.h"
#include <stdio.h>

RobotTab::RobotTab(RobotConfig *new_robot_config, QWidget *parent) :
	QWidget(parent), raw_data_display(NULL), processed_data_display(NULL)
{
	robot_config = new_robot_config;
	use_keyboard = false;

    raw_data_display = new DisplayPane;
    raw_data_display->setTitle("Raw Data");

    RobotCamera *cam = robot_config->sensors.cameras;
    while(cam != NULL)
    {
        raw_data_display->addSource(cam->name);
        cam = cam->next;
    }

    RobotLaser *laser = robot_config->sensors.lasers;
    while(laser != NULL)
    {
        raw_data_display->addSource(laser->name);
        laser = laser->next;
    }

    // Create processed display pane if the config file has maps
    if(robot_config->processedData.maps || robot_config->processedData.images)
    {
        processed_data_display = new DisplayPane;
        processed_data_display->setTitle("Processed Data");

        RobotMap *map = robot_config->processedData.maps;
        while(map != NULL)
        {
            processed_data_display->addSource(map->name);
            map = map->next;
        }

        RobotCamera *image = robot_config->processedData.images;
        while(image != NULL)
        {
            processed_data_display->addSource(image->name);
            image = image->next;
        }
    }


	node_manager = new NodeManager(robot_config);

    setupDataPane();

	/**
     * Connections
     **/
	connect(raw_data_display, SIGNAL(changeSource(const std::string &)),
		node_manager, SLOT(changeRawDataSource(const std::string &)));
    if(processed_data_display)
        connect(processed_data_display, SIGNAL(changeSource(const std::string &)),
            node_manager, SLOT(changeProcessedDataSource(const std::string &)));
	connect(node_manager, SIGNAL(connectionStatusChanged(int)),
		this, SLOT(updateConnectionStatus(int)));
	if(node_manager->camera_node)
		connect(node_manager->camera_node, SIGNAL(frameReceived(const QImage &)),
			raw_data_display, SLOT(setImage(const QImage &)));
    if(node_manager->image_node)
        connect(node_manager->image_node, SIGNAL(frameReceived(const QImage &)),
            processed_data_display, SLOT(setImage(const QImage &)));
	if(node_manager->laser_node)
		connect(node_manager->laser_node, SIGNAL(laserScanReceived(const QImage &, int)),
			raw_data_display, SLOT(setImage(const QImage &, int)));
	if(node_manager->map_node)
        connect(node_manager->map_node, SIGNAL(mapReceived(const QImage &)),
            processed_data_display, SLOT(setImage(const QImage &)));
    if(node_manager->diagnostic_node && robot_config->diagnostics.batteryLevel)
		connect(node_manager->diagnostic_node, SIGNAL(diagnosticDataReceived(const QString &, const QString &)),
			this, SLOT(processDiagnostic(const QString &, const QString &)));
    if(node_manager->range_node)
    {
        data_pane->showRangeLabel(true);
        connect(node_manager->range_node, SIGNAL(rangeReceived(float, bool)),
            data_pane, SLOT(updateRange(float)));
    }


	createLayout();
	setLayout(display_layout);
}

/******************************************************************************
 * Function:    ~RobotTab
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     None
 * Description: Deconstructor:
 *****************************************************************************/
RobotTab::~RobotTab()
{
	delete data_pane;

    if(raw_data_display)
	    delete raw_data_display;

    if(processed_data_display)
        delete processed_data_display;
	
    delete node_manager;
}

/******************************************************************************
 * Function:    robotConnected
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     bool - true if the robot is connected, otherwise false.
 * Description: Returns the connection state of the Control Panel with the
 *              loaded robot.
 *****************************************************************************/
bool RobotTab::robotConnected()
{
	return node_manager->isRunning();
}

/******************************************************************************
 * Function:    disconnectRobot
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     void
 * Description: Stops the connection thread with the robot.
 *****************************************************************************/
void RobotTab::disconnectRobot()
{
	node_manager->stop();
}

/******************************************************************************
 * Function:    connectToRobot
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     void
 * Description: Starts the connection thread with the robot.
 *****************************************************************************/
void RobotTab::connectToRobot()
{
	node_manager->start();//QThread::NormalPriority);
}

/******************************************************************************
 * Function:    getConfig
 * Author:      Scott Logan
 * Parameters:  None
 * Returns:     struct RobotConfig * -
 * Description: Returns the loaded robot's configuration file.
 *****************************************************************************/
struct RobotConfig * RobotTab::getConfig()
{
	return robot_config;
}

/******************************************************************************
 * Function:    createLayout
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     void
 * Description: Creates the display panes layout for the tab.
 *****************************************************************************/
void RobotTab::createLayout()
{
	QFrame *horizontal_separator = new QFrame;
	horizontal_separator->setFrameShape(QFrame::HLine);
	horizontal_separator->setFrameShadow(QFrame::Sunken);

	QFrame *vertical_separator = new QFrame;
	vertical_separator->setFrameShape(QFrame::VLine);
	vertical_separator->setFrameShadow(QFrame::Sunken);

	QHBoxLayout *video_map_layout = new QHBoxLayout;
	video_map_layout->addWidget(raw_data_display);
    // @todo
    // If there is processed items in config file, change below
    if(processed_data_display)
    {
	    video_map_layout->addWidget(vertical_separator);
	    video_map_layout->addWidget(processed_data_display);
    }

	display_layout = new QVBoxLayout;
    //display_layout->setSpacing(0);
	display_layout->addWidget(data_pane);
	display_layout->addWidget(horizontal_separator);
	display_layout->addLayout(video_map_layout);
}

/******************************************************************************
 * Function:    updateConnectionStatus
 * Author:      Matt Richard
 * Parameters:  int status - 
 * Returns:     void
 * Description: 
 *****************************************************************************/
void RobotTab::updateConnectionStatus(int status)
{
    data_pane->connectionStatusChanged(status);

    if(raw_data_display)
        raw_data_display->connectionStatusChanged(status);

    if(processed_data_display)
	    processed_data_display->connectionStatusChanged(status);

	emit connectionStatusChanged(status, robot_config->getRobotName());
}

/******************************************************************************
 * Function:    keyPressEvent
 * Author:      Matt Richard
 * Parameters:  QKeyEvent *event - 
 * Returns:     void
 * Description: Overloaded function.
 *****************************************************************************/
void RobotTab::keyPressEvent(QKeyEvent *event)
{
	// Ignore the event if the robot is not connect or the key is auto repeat
	if(!use_keyboard || !node_manager->isConnected() || event->isAutoRepeat()
        || !node_manager->control_node)
	{
		event->ignore();
		return;
	}

	switch(event->key())
	{
		case Qt::Key_W: // Move forward
			node_manager->control_node->setLinearX(1.0);
			break;

		case Qt::Key_S: // Move backward
			node_manager->control_node->setLinearX(-1.0);
			break;

		case Qt::Key_A: // Turn left
			node_manager->control_node->setAngularZ(1.0);
			break;

		case Qt::Key_D: // Turn right
			node_manager->control_node->setAngularZ(-1.0);
			break;

        case Qt::Key_Up:
            node_manager->control_node->setLinearZ(1.0);
            break;

        case Qt::Key_Down:
            node_manager->control_node->setLinearZ(-1.0);
            break;

        case Qt::Key_Right:
            node_manager->control_node->setLinearY(1.0);
            break;

        case Qt::Key_Left:
            node_manager->control_node->setLinearY(-1.0);
            break;

		default:
			event->ignore();
			return;
	}

	event->accept();
}

/******************************************************************************
 * Function:    keyReleaseEvent
 * Author:      Matt Richard
 * Parameters:  QKeyEvent *event - 
 * Returns:     void
 * Description: Overloaded function.
 *****************************************************************************/
void RobotTab::keyReleaseEvent(QKeyEvent *event)
{
	// Ignore the event if the robot is not connect or the key is auto repeat
	if(!use_keyboard || !node_manager->isConnected() || event->isAutoRepeat()
        || !node_manager->control_node)
	{
		event->ignore();
		return;
	}

	switch(event->key())
	{
		case Qt::Key_W: // Stop forward or backward movement
		case Qt::Key_S:
			node_manager->control_node->setLinearX(0.0);
			break;

		case Qt::Key_A: // Stop rotational movement
		case Qt::Key_D:
			node_manager->control_node->setAngularZ(0.0);
			break;

        case Qt::Key_Up:
        case Qt::Key_Down:
            node_manager->control_node->setLinearX(0.0);
            break;

        case Qt::Key_Right:
        case Qt::Key_Left:
            node_manager->control_node->setLinearY(0.0);
            break;

		default:
			event->ignore();
			return;
	}

	event->accept();
}

void RobotTab::setRCMode(int rc_mode)
{
    if(rc_mode == Globals::Disabled)
    {
        use_keyboard = false;
        if(robot_config->controls.used)
            node_manager->joystick_node->unsubscribe();

        data_pane->setRCModeText("Disabled");
    }
    else if(rc_mode == Globals::Keyboard)
    {
        use_keyboard = true;
        if(robot_config->controls.used)
            node_manager->joystick_node->unsubscribe();

        data_pane->setRCModeText("Keyboard");
    }
    else if(rc_mode == Globals::Joystick)
    {
        use_keyboard = false;
        if(robot_config->controls.used)
            node_manager->joystick_node->subscribe();

        data_pane->setRCModeText("Joystick");
    }
    else
        printf("Unknown RC mode: %d\n", rc_mode);
}

/******************************************************************************
 * Function:    isKeyboardEnabled
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     bool - True is keyboard is enabled, otherwise false
 * Description: Returns the state of keyboard remote control.
 *****************************************************************************/
bool RobotTab::isKeyboardEnabled() const
{
	return use_keyboard;
}

/******************************************************************************
 * Function:    processDiagnostic
 * Author:      Scott K Logan
 * Parameters:  QString key - type of diagnostic
 *              QString val - value reported
 * Returns:     void
 * Description: Processes an incoming diagnostic
 *****************************************************************************/
void RobotTab::processDiagnostic(const QString &key, const QString &val)
{
	RobotBatteryLevel *temp_batteryLevel = robot_config->diagnostics.batteryLevel;
	while(temp_batteryLevel != NULL)
	{
		if(temp_batteryLevel->name == key)
			data_pane->updateBatteryData(val.toFloat());
		temp_batteryLevel = temp_batteryLevel->next;
	}
	RobotVoltage *temp_voltage = robot_config->diagnostics.voltage;
	while(temp_voltage != NULL)
	{
		if(temp_voltage->name == key)
		{
			// Do Something
		}
		temp_voltage = temp_voltage->next;
	}
}


void RobotTab::setupDataPane()
{
    data_pane = new DataPane;

    RobotOdometry *odom = robot_config->processedData.odometry;
    while(odom != NULL)
    {
        connect(
            node_manager->addOdometryNode(odom->topicName.toStdString()),
            SIGNAL(odometryDataReceived(const QVector3D &, const QQuaternion &,
                                        const QVector3D &, const QVector3D &)),
            data_pane->addOdometryDisplay(odom->name, odom->position,
                                          odom->orientation, odom->linearVelocity,
                                          odom->angularVelocity, !odom->hideAttitude,
                                          !odom->hideHeading),
            SLOT(updateOdometryDisplay(const QVector3D &, const QQuaternion &,
                                       const QVector3D &, const QVector3D &))
            );
        odom = odom->next;
    }

    // Create nodes and widgets for all imu sensors
    RobotIMU *imu = robot_config->sensors.imu;
    while(imu != NULL)
    {
        connect(
            node_manager->addImuNode(imu->topicName.toStdString()),
            SIGNAL(imuDataReceived(const QQuaternion &, const QVector3D &,
                                   const QVector3D &)),
            data_pane->addImuDisplay(imu->name, imu->roll, imu->pitch,
                                     imu->yaw, imu->angularVelocity, imu->linearAcceleration,
                                     !imu->hideAttitude, !imu->hideHeading),
            SLOT(updateImuDisplay(const QQuaternion &, const QVector3D &,
                                  const QVector3D &))
            );
        imu = imu->next;
    }


    qRegisterMetaType< std::vector<double> >("std::vector<double>");
    RobotJoint *joint = robot_config->joint_states.joints;
    if(robot_config->joint_states.used)
    {
        // Group all joints and their display names in string lists
        QStringList name_list;
        QStringList disp_name_list;
        while(joint != NULL)
        {
            name_list << joint->name;
            disp_name_list << joint->displayName;
            joint = joint->next;
        }

        node_manager->joint_node->setTopic(robot_config->joint_states.topicName.toStdString());
        connect(
            node_manager->joint_node,
            SIGNAL(jointDataReceived(const QStringList &, const std::vector<double> &,
                                     const std::vector<double> &, const std::vector<double> &)),
            data_pane->addJointStateDisplay("Joints", name_list, disp_name_list,
                                            robot_config->joint_states.position,
                                            robot_config->joint_states.velocity,
                                            robot_config->joint_states.effort),
            SLOT(updateJointStateDisplay(const QStringList &, const std::vector<double> &,
                                         const std::vector<double> &, const std::vector<double> &))
            );
    }

    // Create nodes and widgets for all gps sensors
    RobotGPS *gps = robot_config->sensors.gps;
    while(gps != NULL)
    {
        connect(node_manager->addGpsNode(gps->topicName.toStdString()),
                SIGNAL(gpsDataReceived(double, double, double)),
                data_pane->addGpsDisplay(gps->name, gps->latitude,
                    gps->longitude, gps->altitude),
                SLOT(updateGpsDisplay(double, double, double)));
        gps = gps->next;
    }
}
