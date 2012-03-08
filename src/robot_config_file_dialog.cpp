/* @todo Add license here */

/**
 * \file   robot_config_file_dialog.cpp
 * \date   Dec 6, 2011
 * \author Matt Richard, Scott Logan
 */
#include <QtGui>
#include "control_panel/robot_config_file_dialog.h"
#include "stdio.h"

////////////////////////// General Tab ///////////////////////////////

GeneralTab::GeneralTab(struct RobotConfig *robot_config, QWidget *parent)
    : QWidget(parent)
{
    QLabel *robot_name_label = new QLabel(tr("Robot's Name"));
    QLineEdit *robot_name_lineedit = new QLineEdit(robot_config->robotName);

    QLabel *system_label = new QLabel(tr("System"));
    QStringList system_list;
    system_list << "UGV (Unmanned Ground Vehicle)"
                << "UAV (Unmanned Aerial Vehicle)"
                << "AUV (Autonomous Underwater Vehicle)"
                << "USV (Unmanned Surface Vehicle)"
                << "Humanoid";
    QComboBox *system_combobox = new QComboBox;
    system_combobox->addItems(system_list);

    int index = system_list.indexOf(robot_config->system);
    if(index != -1)
        system_combobox->setCurrentIndex(index);

    QLabel *drive_system_label = new QLabel(tr("Drive System"));
    QLineEdit *drive_system_lineedit = new QLineEdit(robot_config->driveSystem);

    QLabel *image_file_label = new QLabel(tr("Image File"));
    QLineEdit *image_file_lineedit = new QLineEdit(robot_config->imageFilePath);
    QPushButton *browse_button = new QPushButton(tr("Browse"));

    QLabel *namespace_label = new QLabel(tr("Namespace"));
    QLineEdit *namespace_lineedit = new QLineEdit(robot_config->nameSpace);

    QHBoxLayout *image_file_hlayout = new QHBoxLayout;
    image_file_hlayout->addWidget(image_file_lineedit);
    image_file_hlayout->addWidget(browse_button);

    QGridLayout *general_tab_layout = new QGridLayout;
    general_tab_layout->addWidget(robot_name_label, 0, 0);
    general_tab_layout->addWidget(robot_name_lineedit, 0, 1);
    general_tab_layout->addWidget(system_label, 1, 0);
    general_tab_layout->addWidget(system_combobox, 1, 1);
    general_tab_layout->addWidget(drive_system_label, 2, 0);
    general_tab_layout->addWidget(drive_system_lineedit, 2, 1);
    general_tab_layout->addWidget(image_file_label, 3, 0);
    general_tab_layout->addLayout(image_file_hlayout, 3, 1);
    general_tab_layout->addWidget(namespace_label, 4, 0);
    general_tab_layout->addWidget(namespace_lineedit, 4, 1);
    setLayout(general_tab_layout);
}

////////////////////////////// Sensors Tab ///////////////////////////////

SensorsTab::SensorsTab(struct RobotConfig *robot_config, QWidget *parent)
    : QWidget(parent)
{
    QLabel *sensors_label = new QLabel(tr("Sensors"));
    QStringList sensor_list;
    sensor_list << "Camera (Image.msg)"
                << "Compass (Imu.msg)"
                << "GPS (NavSatFix.msg)"
                << "IMU (Imu.msg)"
                << "Laser Rangefinder (LaserScan.msg)"
                << "Sonar/1D-Infrared (Range.msg)";
    QComboBox *sensors_combobox = new QComboBox;
    sensors_combobox->addItems(sensor_list);
    QPushButton *add_sensor_button = new QPushButton(tr("Add"));


    /* Add Camera config data to camera item list */
    QList<QTreeWidgetItem *> camera_itemlist;
    for(unsigned int i = 0; i < robot_config->sensors.cameras.size(); i++)
    {
        QTreeWidgetItem *camera_item = new QTreeWidgetItem;
        camera_item->setText(0, tr("Camera (Image.msg)"));
        camera_item->setText(1, robot_config->sensors.cameras[i].name);
        camera_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Name") << robot_config->sensors.cameras[i].name)));
        camera_item->addChild(new QTreeWidgetItem((QTreeWidget *)0, 
            (QStringList() << tr("Name") << robot_config->sensors.cameras[i].topicName)));

        camera_itemlist.append(camera_item);
    }

    /* @todo Add compass' */

    /* Add GPS's config data to GPS list */
    QList<QTreeWidgetItem *> gps_itemlist;
    for(unsigned int i = 0; i < robot_config->sensors.gps.size(); i++)
    {
        QTreeWidgetItem *gps_item = new QTreeWidgetItem;
        gps_item->setText(0, tr("GPS (NavSatFix.msg)"));
        gps_item->setText(1, robot_config->sensors.gps[i].name);
        gps_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Name") << robot_config->sensors.gps[i].name)));
        gps_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Topic Name") << robot_config->sensors.gps[i].topicName)));

        QString gps_lat("No");
        if(robot_config->sensors.gps[i].latitude)
            gps_lat = "Yes";
        gps_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Latitude") << gps_lat)));

        QString gps_long("No");
        if(robot_config->sensors.gps[i].longitude)
            gps_long = "Yes";
        gps_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Longitude") << gps_long)));

        QString gps_alt("No");
        if(robot_config->sensors.gps[i].altitude)
            gps_alt = "Yes";
        gps_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Altitude") << gps_alt)));

        gps_itemlist.append(gps_item);
    }

    /* Add IMU config data to IMU item list */
    QList<QTreeWidgetItem *> imu_itemlist;
    for(unsigned int i = 0; i < robot_config->sensors.imu.size(); i++)
    {
        QTreeWidgetItem *imu_item = new QTreeWidgetItem;
        imu_item->setText(0, tr("IMU (Imu.msg)"));
        imu_item->setText(1, robot_config->sensors.imu[i].name);
        imu_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Name") << robot_config->sensors.imu[i].name)));
        imu_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Topic Name") << robot_config->sensors.imu[i].topicName)));

        QString imu_roll("No");
        if(robot_config->sensors.imu[i].roll)
            imu_roll = "Yes";
        imu_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Roll") << imu_roll)));

        QString imu_pitch("No");
        if(robot_config->sensors.imu[i].pitch)
            imu_pitch = "Yes";
        imu_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Pitch") << imu_pitch)));

        QString imu_yaw("No");
        if(robot_config->sensors.imu[i].yaw)
            imu_yaw = "Yes";
        imu_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Yaw") << imu_yaw)));

        QString imu_ang_vel("No");
        if(robot_config->sensors.imu[i].angularVelocity)
            imu_ang_vel = "Yes";
        imu_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Angular Velocity") << imu_ang_vel)));

        QString imu_lin_accel("No");
        if(robot_config->sensors.imu[i].linearAcceleration)
            imu_lin_accel = "Yes";
        imu_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Linear Acceleration") << imu_lin_accel)));

        QString imu_show_attitude("No");
        if(!robot_config->sensors.imu[i].hideAttitude)
            imu_show_attitude = "Yes";
        imu_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Show Attitude Indicator") << imu_show_attitude)));

        QString imu_show_heading("No");
        if(!robot_config->sensors.imu[i].hideHeading)
            imu_show_heading = "Yes";
        imu_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Show Heading Indicator") << imu_show_heading)));

        QString imu_show_labels("No");
        if(!robot_config->sensors.imu[i].hideLabels)
            imu_show_labels = "Yes";
        imu_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Show Labels") << imu_show_labels)));

        imu_itemlist.append(imu_item);
    }

    /* Add Laser config data to laser item list */
    QList<QTreeWidgetItem *> laser_itemlist;
    for(unsigned int i = 0; i < robot_config->sensors.lasers.size(); i++)
    {
        QTreeWidgetItem *laser_item = new QTreeWidgetItem;
        laser_item->setText(0, tr("Laser Rangefinder (LaserScan.msg)"));
        laser_item->setText(1, robot_config->sensors.lasers[i].name);
        laser_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Name") << robot_config->sensors.lasers[i].name)));
        laser_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Topic Name") << robot_config->sensors.lasers[i].topicName)));

        laser_itemlist.append(laser_item);
    }

    /* Add Range config data to range item list */
    QList<QTreeWidgetItem *> range_itemlist;
    for(unsigned int i = 0; i < robot_config->sensors.range.size(); i++)
    {
        QTreeWidgetItem *range_item = new QTreeWidgetItem;
        range_item->setText(0, tr("Sonar/1D-Infrared (Range.msg)"));
        range_item->setText(1, robot_config->sensors.range[i].name);
        range_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Name") << robot_config->sensors.range[i].name)));
        range_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Topic Name") << robot_config->sensors.range[i].topicName)));

        range_itemlist.append(range_item);
    }

    QStringList column_list;
    column_list << "Sensors" << "Value";

    /* Create Tree Widget */
    QTreeWidget *sensors_treewidget = new QTreeWidget;
    sensors_treewidget->setHeaderLabels(column_list);
    sensors_treewidget->addTopLevelItems(camera_itemlist);
    sensors_treewidget->addTopLevelItems(gps_itemlist);
    sensors_treewidget->addTopLevelItems(imu_itemlist);
    sensors_treewidget->addTopLevelItems(laser_itemlist);
    sensors_treewidget->addTopLevelItems(range_itemlist);


    QHBoxLayout *sensors_hlayout = new QHBoxLayout;
    sensors_hlayout->addWidget(sensors_label, Qt::AlignLeft);
    sensors_hlayout->addStretch();
    sensors_hlayout->addWidget(sensors_combobox, Qt::AlignRight);
    sensors_hlayout->addWidget(add_sensor_button, Qt::AlignRight);

    QVBoxLayout *sensors_tab_layout = new QVBoxLayout;
    sensors_tab_layout->addLayout(sensors_hlayout);
    sensors_tab_layout->addWidget(sensors_treewidget);
    setLayout(sensors_tab_layout);
}

////////////////////////////// Processed Data Tab //////////////////////////

ProcessedDataTab::ProcessedDataTab(QWidget *parent) : QWidget(parent)
{
    QLabel *processed_data_label = new QLabel(tr("Processed Data"));
    QStringList processed_data_list;
    processed_data_list << "Disparity Image (DisparityImage.msg)"
                        << "Processed Image (Image.msg)"
                        << "Map (Map.msg)"
                        << "Odometry (Odometry.msg)";
    QComboBox *processed_data_combobox = new QComboBox;
    processed_data_combobox->addItems(processed_data_list);
    QPushButton *add_button = new QPushButton(tr("Add"));

    QScrollArea *processed_data_scrollarea = new QScrollArea;

    QHBoxLayout *processed_data_hlayout = new QHBoxLayout;
    processed_data_hlayout->addWidget(processed_data_label, Qt::AlignLeft);
    processed_data_hlayout->addStretch();
    processed_data_hlayout->addWidget(processed_data_combobox, Qt::AlignRight);
    processed_data_hlayout->addWidget(add_button, Qt::AlignRight);

    QVBoxLayout *processed_data_layout = new QVBoxLayout;
    processed_data_layout->addLayout(processed_data_hlayout);
    processed_data_layout->addWidget(processed_data_scrollarea);
    setLayout(processed_data_layout);
}

///////////////////////// Joints Tab ////////////////////////////

JointsTab::JointsTab(QWidget *parent) : QWidget(parent)
{
    QLabel *topic_name_label = new QLabel(tr("Topic Name"));
    QLineEdit *topic_name_lineedit = new QLineEdit;

    QCheckBox *position_checkbox = new QCheckBox(tr("Position"));
    position_checkbox->setChecked(true);
    QCheckBox *velocity_checkbox = new QCheckBox(tr("Velocity"));
    velocity_checkbox->setChecked(true);
    QCheckBox *effort_checkbox = new QCheckBox(tr("Effort"));

    QLabel *joint_states_label = new QLabel(tr("Joints"));
    QPushButton *add_button = new QPushButton(tr("Add"));

    QScrollArea *joints_scrollarea = new QScrollArea;

    QGridLayout *joints_gridlayout = new QGridLayout;
    joints_gridlayout->addWidget(topic_name_label, 0, 0);
    joints_gridlayout->addWidget(topic_name_lineedit, 0, 1);
    joints_gridlayout->addWidget(position_checkbox, 1, 1);
    joints_gridlayout->addWidget(velocity_checkbox, 2, 1);
    joints_gridlayout->addWidget(effort_checkbox, 3, 1);
    joints_gridlayout->addWidget(joint_states_label, 4, 0);
    joints_gridlayout->addWidget(add_button, 4, 1, Qt::AlignRight);

    QVBoxLayout *joints_layout = new QVBoxLayout;
    joints_layout->addLayout(joints_gridlayout);
    joints_layout->addWidget(joints_scrollarea);
    setLayout(joints_layout);
}

////////////////////////////// Controls Tab ////////////////////////////

ControlsTab::ControlsTab(QWidget *parent) : QWidget(parent)
{
    QLabel *controls_label = new QLabel(tr("Controls"));
    QStringList controls_list;
    controls_list << "Teleoperation (Twist.msg)";
    QComboBox *controls_combobox = new QComboBox;
    controls_combobox->addItems(controls_list);
    QPushButton *add_button = new QPushButton(tr("Add"));

    QScrollArea *controls_scrollarea = new QScrollArea;

    QHBoxLayout *controls_hlayout = new QHBoxLayout;
    controls_hlayout->addWidget(controls_label, Qt::AlignLeft);
    controls_hlayout->addStretch();
    controls_hlayout->addWidget(controls_combobox, Qt::AlignRight);
    controls_hlayout->addWidget(add_button, Qt::AlignRight);

    QVBoxLayout *controls_layout = new QVBoxLayout;
    controls_layout->addLayout(controls_hlayout);
    controls_layout->addWidget(controls_scrollarea);
    setLayout(controls_layout);
}

//////////////////////////// Services Tab ///////////////////////////////

ServicesTab::ServicesTab(QWidget *parent) : QWidget(parent)
{
    QLabel *services_label = new QLabel(tr("Services"));
    QStringList services_list;
    services_list << "Custom (Empty.srv)"
                  << "Land (Empty.srv)"
                  << "Takeoff (Empty.srv)";
    QComboBox *services_combobox = new QComboBox;
    services_combobox->addItems(services_list);
    QPushButton *add_button = new QPushButton(tr("Add"));

    QScrollArea *services_scrollarea = new QScrollArea;

    QHBoxLayout *services_hlayout = new QHBoxLayout;
    services_hlayout->addWidget(services_label, Qt::AlignLeft);
    services_hlayout->addStretch();
    services_hlayout->addWidget(services_combobox, Qt::AlignRight);
    services_hlayout->addWidget(add_button, Qt::AlignRight);

    QVBoxLayout *services_layout = new QVBoxLayout;
    services_layout->addLayout(services_hlayout);
    services_layout->addWidget(services_scrollarea);
    setLayout(services_layout);
}

////////////////////////// Diagnostics Tab ////////////////////////////////

DiagnosticsTab::DiagnosticsTab(QWidget *parent) : QWidget(parent)
{
    QLabel *topic_name_label = new QLabel(tr("Topic Name"));
    QLineEdit *topic_name_lineedit = new QLineEdit;

    QLabel *path_label = new QLabel(tr("Path"));
    QLineEdit *path_lineedit = new QLineEdit;

    QLabel *diagnostics_label = new QLabel(tr("Diagnostics"));
    QStringList diagnostics_list;
    diagnostics_list << "Battery Level"
                     << "Diagnostic";
    QComboBox *diagnostics_combobox = new QComboBox;
    diagnostics_combobox->addItems(diagnostics_list);
    QPushButton *add_button = new QPushButton(tr("Add"));

    QScrollArea *diagnostics_scrollarea = new QScrollArea;

    QHBoxLayout *add_diagnostic_hlayout = new QHBoxLayout;
    add_diagnostic_hlayout->addWidget(diagnostics_combobox);
    add_diagnostic_hlayout->addWidget(add_button);

    QGridLayout *diagnostics_gridlayout = new QGridLayout;
    diagnostics_gridlayout->addWidget(topic_name_label, 0, 0);
    diagnostics_gridlayout->addWidget(topic_name_lineedit, 0, 1);
    diagnostics_gridlayout->addWidget(path_label, 1, 0);
    diagnostics_gridlayout->addWidget(path_lineedit, 1, 1);
    diagnostics_gridlayout->addWidget(diagnostics_label, 2, 0);
    diagnostics_gridlayout->addLayout(add_diagnostic_hlayout, 2, 1);
    
    QVBoxLayout *diagnosticstab_layout = new QVBoxLayout;
    diagnosticstab_layout->addLayout(diagnostics_gridlayout);
    diagnosticstab_layout->addWidget(diagnostics_scrollarea);
    setLayout(diagnosticstab_layout);
}

/////////////////////// Robot Config File Dialog //////////////////////////

/******************************************************************************
 * Function:    RobotConfigFileDialog
 * Author:      Matt Richard, Scott Logan
 * Parameters:  struct RobotConfig *new_robot_config -
 *              QWidget *parent - 
 * Returns:     None
 * Description: Constructor.
 *****************************************************************************/
RobotConfigFileDialog::RobotConfigFileDialog(
	struct RobotConfig *new_robot_config, QWidget *parent) : QDialog(parent)
{
	robot_config = new_robot_config;

    tab_widget = new QTabWidget;
    tab_widget->addTab(new GeneralTab(robot_config), tr("General"));
    tab_widget->addTab(new SensorsTab(robot_config), tr("Sensors"));
    tab_widget->addTab(new ProcessedDataTab, tr("ProcessedData"));
    tab_widget->addTab(new JointsTab, tr("Joints"));
    tab_widget->addTab(new ControlsTab, tr("Controls"));
    tab_widget->addTab(new ServicesTab, tr("Services"));
    tab_widget->addTab(new DiagnosticsTab, tr("Diagnostics"));

    button_box = new QDialogButtonBox(QDialogButtonBox::Cancel | QDialogButtonBox::Save);

	//createWidgets();
	//createLayout();

	// Connections
	connect(button_box, SIGNAL(accepted()), this, SLOT(accept()));
	connect(button_box, SIGNAL(rejected()), this, SLOT(reject()));
	//connect(add_button, SIGNAL(clicked()), this, SLOT(addSensor()));
	//connect(edit_button, SIGNAL(clicked()), this, SLOT(editSelectedSensor()));
	//connect(remove_button, SIGNAL(clicked()), this, SLOT(removeSelectedSensors()));

    QVBoxLayout *dialog_layout = new QVBoxLayout;
    dialog_layout->setSizeConstraint(QLayout::SetNoConstraint);
    dialog_layout->addWidget(tab_widget);
    dialog_layout->addWidget(button_box);
	setLayout(dialog_layout);
	//layout()->setSizeConstraint(QLayout::SetFixedSize);
}

/******************************************************************************
 * Function:    create_widgets
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     void
 * Description: 
 *****************************************************************************/
void RobotConfigFileDialog::createWidgets()
{
/*
	robot_name_label = new QLabel(tr("Robot's Name"));
	system_label = new QLabel(tr("System"));
	drive_system_label = new QLabel(tr("Drive System"));
	image_file_label = new QLabel(tr("Image File"));
	sensors_label = new QLabel(tr("Sensors/Components"));

	robot_name_lineedit = new QLineEdit(robot_config->robotName);
	drive_system_lineedit = new QLineEdit;
	image_file_lineedit = new QLineEdit;

	QStringList system_list;
	system_list << "UGV (Unmanned Ground Vehicle)"
				<< "UAV (Unmanned Aerial Vehicle)"
                << "AUV (Autonomous Underwater Vehicle)"
				<< "USV (Unmanned Surface Vehicle)"
				<< "Humanoid";
	system_combobox = new QComboBox;
	system_combobox->addItems(system_list);

	QStringList sensors_list;
	sensors_list << "Battery Level" << "Camera" << "Compass" << "Computer" << "Contact"
				 << "Encoder" << "Fan" << "GPS" << "IMU" << "Laser Rangefinder"
				 << "Light" << "Pressure" << "Radiation" << "Sonar" << "Temperature"
				 << "Voltage" << "Velocity";
	sensors_combobox = new QComboBox;
	sensors_combobox->addItems(sensors_list);

	browse_button = new QPushButton(tr("Browse"));
	add_button = new QPushButton(tr("Add"));
	edit_button = new QPushButton(tr("Edit Selected"));
	remove_button = new QPushButton(tr("Remove Selected"));

	sensors_scrollarea = new QScrollArea;
	sensors_scrollarea->setWidgetResizable(true);
	sensors_scrollarea->setFixedHeight(250);

	sensor_list_widget = new QWidget;

	save_cancel_buttonbox = new QDialogButtonBox(QDialogButtonBox::Cancel |
		QDialogButtonBox::Save);
*/
}

/******************************************************************************
 * Function:    create_layout
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     void
 * Description:
 *****************************************************************************/
void RobotConfigFileDialog::createLayout()
{
/*
	QHBoxLayout *robot_name_hlayout = new QHBoxLayout;
	robot_name_hlayout->addWidget(robot_name_label);
	robot_name_hlayout->addWidget(robot_name_lineedit);

	QHBoxLayout *system_hlayout = new QHBoxLayout;
	system_hlayout->addWidget(system_label);
	system_hlayout->addWidget(system_combobox);

	QHBoxLayout *drive_system_hlayout = new QHBoxLayout;
	drive_system_hlayout->addWidget(drive_system_label);
	drive_system_hlayout->addWidget(drive_system_lineedit);
*/
/*
	QHBoxLayout *image_file_hlayout = new QHBoxLayout;
//	image_file_hlayout->addWidget(image_file_label);
	image_file_hlayout->addWidget(image_file_lineedit);
	image_file_hlayout->addWidget(browse_button);

	QHBoxLayout *sensors_hlayout = new QHBoxLayout;
	sensors_hlayout->addWidget(sensors_combobox);
	sensors_hlayout->addStretch();
	sensors_hlayout->addWidget(add_button, 0, Qt::AlignRight);

	QHBoxLayout *edit_remove_hlayout = new QHBoxLayout;
	edit_remove_hlayout->addStretch();
	edit_remove_hlayout->addWidget(edit_button);
	edit_remove_hlayout->addWidget(remove_button);

	sensor_list_layout = new QVBoxLayout;
	sensor_list_layout->addStretch();
	sensor_list_layout->setSpacing(0);

	sensor_list_widget->setLayout(sensor_list_layout);
	sensors_scrollarea->setWidget(sensor_list_widget);

	dialog_layout = new QGridLayout;
	dialog_layout->addWidget(robot_name_label, 0, 0);
	dialog_layout->addWidget(robot_name_lineedit, 0, 1);
	dialog_layout->addWidget(system_label, 1, 0);
	dialog_layout->addWidget(system_combobox, 1, 1);
	dialog_layout->addWidget(drive_system_label, 2, 0);
	dialog_layout->addWidget(drive_system_lineedit, 2, 1);
	dialog_layout->addWidget(image_file_label, 3, 0);
	dialog_layout->addLayout(image_file_hlayout, 3, 1);
	dialog_layout->addWidget(sensors_label, 4, 0);
	dialog_layout->addLayout(sensors_hlayout, 4, 1);
	dialog_layout->addLayout(edit_remove_hlayout, 5, 1, Qt::AlignRight);
	dialog_layout->addWidget(sensors_scrollarea, 6, 0, 1, 2);
	dialog_layout->addWidget(save_cancel_buttonbox, 7, 1, Qt::AlignRight);
*/
}


void RobotConfigFileDialog::setTitle(const std::string &title)
{
	setWindowTitle(title.c_str());
}

void RobotConfigFileDialog::addSensor()
{
/*
	AddSensorDialog add_sensor_dialog(sensors_combobox->currentText(),
		this);

	if(add_sensor_dialog.exec())
	{
		sensor_widget = new RobotConfigSensorWidget;
		sensor_widget->setSensorType(sensors_combobox->currentText());
		sensor_widget->setSensorName(add_sensor_dialog.getName());

		sensor_list_layout->removeItem(sensor_list_layout->takeAt(sensor_list_layout->count() - 1));
		sensor_list_layout->addWidget(sensor_widget);
		sensor_list_layout->addStretch();
	}
*/
}

void RobotConfigFileDialog::editSelectedSensor()
{

}

void RobotConfigFileDialog::removeSelectedSensors()
{

}
