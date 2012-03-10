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
 * \file   robot_config_file_dialog.cpp
 * \date   Dec 6, 2011
 * \author Matt Richard, Scott Logan
 */
#include <QtGui>
#include <iostream>
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
    system_list << "UGV"// (Unmanned Ground Vehicle)"
                << "UAV"// (Unmanned Aerial Vehicle)"
                << "AUV"// (Autonomous Underwater Vehicle)"
                << "USV"// (Unmanned Surface Vehicle)"
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
    sensors_combobox = new QComboBox;
    sensors_combobox->addItems(sensor_list);
    QPushButton *add_sensor_button = new QPushButton(tr("Add"));
    connect(add_sensor_button, SIGNAL(clicked()), this, SLOT(addSensor()));


    /* Add Camera config data to camera item list */
    QList<QTreeWidgetItem *> camera_itemlist;
    for(unsigned int i = 0; i < robot_config->sensors.cameras.size(); i++)
    {
        QTreeWidgetItem *camera_item = new QTreeWidgetItem;
        camera_item->setText(0, tr("Camera"));
        camera_item->setText(1, robot_config->sensors.cameras[i].name);
        camera_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Name") << robot_config->sensors.cameras[i].name)));
        camera_item->addChild(new QTreeWidgetItem((QTreeWidget *)0, 
            (QStringList() << tr("Topic Name") << robot_config->sensors.cameras[i].topicName)));

        camera_itemlist.append(camera_item);
    }

    /* @todo Add compass' */

    /* Add GPS's config data to GPS list */
    QList<QTreeWidgetItem *> gps_itemlist;
    for(unsigned int i = 0; i < robot_config->sensors.gps.size(); i++)
    {
        QTreeWidgetItem *gps_item = new QTreeWidgetItem;
        gps_item->setText(0, tr("GPS"));
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
        imu_item->setText(0, tr("IMU"));
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
        laser_item->setText(0, tr("Laser Rangefinder"));
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
        range_item->setText(0, tr("Sonar/1D-Infrared"));
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
    sensors_treewidget = new QTreeWidget;
    sensors_treewidget->setHeaderLabels(column_list);
    sensors_treewidget->addTopLevelItems(camera_itemlist);
    sensors_treewidget->addTopLevelItems(gps_itemlist);
    sensors_treewidget->addTopLevelItems(imu_itemlist);
    sensors_treewidget->addTopLevelItems(laser_itemlist);
    sensors_treewidget->addTopLevelItems(range_itemlist);


    /* Create layout */
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

void SensorsTab::addSensor()
{
    SensorType type = SensorType(sensors_combobox->currentIndex());
    QString type_str;

    if(type == Camera)
        type_str = "Camera";
    else if(type == Compass)
        type_str = "Compass";
    else if(type == Gps)
        type_str = "GPS";
    else if(type == Imu)
        type_str = "IMU";
    else if(type == Laser)
        type_str = "Laser Rangefinder";
    else if(type == Range)
        type_str = "Sonar/1D-Infrared";
    else
    {
        std::cerr << "ERROR -- Unknown SensorType '" << type << "' encountered"
                  << " while adding sensor to configuration file" << std::endl;
        return;
    }

    if(type == Camera || type == Laser || type == Range) // Others require different dialog
    {
        ComponentDialog component_dialog(this);
        component_dialog.setWindowTitle(QString("Add %1").arg(type_str));

        if(component_dialog.exec())
        {
            QTreeWidgetItem *item = new QTreeWidgetItem;
            item->setText(0, type_str);
            item->setText(1, component_dialog.getName());
            item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
                (QStringList() << tr("Name") << component_dialog.getName())));
            item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
                (QStringList() << tr("Topic Name") << component_dialog.getTopicName())));

            sensors_treewidget->addTopLevelItem(item);
        }
    }
    else if(type == Compass)
    {

    }
    else if(type == Gps)
    {

    }
    else if(type == Imu)
    {

    }
}

void SensorsTab::editSensor()
{

}


////////////////////////////// Processed Data Tab //////////////////////////
ProcessedDataTab::ProcessedDataTab(struct RobotProcessedData *robot_processed_data,
    QWidget *parent) : QWidget(parent)
{
    QLabel *processed_data_label = new QLabel(tr("Processed Data"));
    QStringList processed_data_list;
    processed_data_list << "Disparity Image (DisparityImage.msg)"
                        << "Map (Map.msg)"
                        << "Odometry (Odometry.msg)"
                        << "Processed Image (Image.msg)";
    processed_data_combobox = new QComboBox;
    processed_data_combobox->addItems(processed_data_list);
    QPushButton *add_button = new QPushButton(tr("Add"));
    connect(add_button, SIGNAL(clicked()), this, SLOT(addProcessedData()));

    /* Add disparity images from robot configuration file */
    QList<QTreeWidgetItem *> disparity_image_itemlist;
    for(unsigned int i = 0; i < robot_processed_data->disparity_images.size(); i++)
    {
        QTreeWidgetItem *disparity_image_item = new QTreeWidgetItem;
        disparity_image_item->setText(0, tr("Disparity Image"));
        disparity_image_item->setText(1, robot_processed_data->disparity_images[i].name);
        disparity_image_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Name") << robot_processed_data->disparity_images[i].name)));
        disparity_image_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Topic Name") << robot_processed_data->disparity_images[i].topicName)));

        disparity_image_itemlist.append(disparity_image_item);
    }

    /* Add odometry from robot configuration file */
    QList<QTreeWidgetItem *> odometry_itemlist;
    for(unsigned int i = 0; i < robot_processed_data->odometry.size(); i++)
    {
        QTreeWidgetItem *odometry_item = new QTreeWidgetItem;
        odometry_item->setText(0, tr("Odometry"));
        odometry_item->setText(1, robot_processed_data->odometry[i].name);
        odometry_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Name") << robot_processed_data->odometry[i].name)));
        odometry_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Topic Name") << robot_processed_data->odometry[i].topicName)));

        QString odom_pos("No");
        if(robot_processed_data->odometry[i].position)
            odom_pos = "Yes";
        odometry_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Position") << odom_pos)));

        QString odom_ori("No");
        if(robot_processed_data->odometry[i].orientation)
            odom_ori = "Yes";
        odometry_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Orientation") << odom_ori)));

        QString odom_lin_vel("No");
        if(robot_processed_data->odometry[i].linearVelocity)
            odom_lin_vel = "Yes";
        odometry_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Linear Velocity") << odom_lin_vel)));

        QString odom_ang_vel("No");
        if(robot_processed_data->odometry[i].angularVelocity)
            odom_ang_vel = "Yes";
        odometry_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Angular Velocity") << odom_ang_vel)));

        QString odom_show_attitude("No");
        if(!robot_processed_data->odometry[i].hideAttitude)
            odom_show_attitude = "Yes";
        odometry_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Show Attitude Indicator") << odom_show_attitude)));

        QString odom_show_heading("No");
        if(!robot_processed_data->odometry[i].hideHeading)
            odom_show_heading = "Yes";
        odometry_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Show Heading Indicator") << odom_show_heading)));

        QString odom_show_labels("No");
        if(!robot_processed_data->odometry[i].hideLabels)
            odom_show_labels = "Yes";
        odometry_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Show Labels") << odom_show_labels)));

        odometry_itemlist.append(odometry_item);
    }

    /* Add maps from robot configuration file */
    QList<QTreeWidgetItem *> map_itemlist;
    for(unsigned int i = 0; i < robot_processed_data->maps.size(); i++)
    {
        QTreeWidgetItem *map_item = new QTreeWidgetItem;
        map_item->setText(0, tr("Map"));
        map_item->setText(1, robot_processed_data->maps[i].name);
        map_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Name") << robot_processed_data->maps[i].name)));
        map_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Topic Name") << robot_processed_data->maps[i].topicName)));

        map_itemlist.append(map_item);
    }

    /* Add processed images from robot configuation file */
    QList<QTreeWidgetItem *> processed_image_itemlist;
    for(unsigned int i = 0; i < robot_processed_data->images.size(); i++)
    {
        QTreeWidgetItem *processed_image_item = new QTreeWidgetItem;
        processed_image_item->setText(0, tr("Processed Image"));
        processed_image_item->setText(1, robot_processed_data->images[i].name);
        processed_image_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Name") << robot_processed_data->images[i].name)));
        processed_image_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << tr("Topic Name") << robot_processed_data->images[i].topicName)));

        processed_image_itemlist.append(processed_image_item);
    }

    QStringList column_list;
    column_list << "Processed Data" << "Values";

    /* Create tree widget */
    processed_data_treewidget = new QTreeWidget;
    processed_data_treewidget->setHeaderLabels(column_list);
    processed_data_treewidget->addTopLevelItems(disparity_image_itemlist);
    processed_data_treewidget->addTopLevelItems(odometry_itemlist);
    processed_data_treewidget->addTopLevelItems(map_itemlist);
    processed_data_treewidget->addTopLevelItems(processed_image_itemlist);

    /* Create layout */
    QHBoxLayout *processed_data_hlayout = new QHBoxLayout;
    processed_data_hlayout->addWidget(processed_data_label, Qt::AlignLeft);
    processed_data_hlayout->addStretch();
    processed_data_hlayout->addWidget(processed_data_combobox, Qt::AlignRight);
    processed_data_hlayout->addWidget(add_button, Qt::AlignRight);

    QVBoxLayout *processed_data_layout = new QVBoxLayout;
    processed_data_layout->addLayout(processed_data_hlayout);
    processed_data_layout->addWidget(processed_data_treewidget);
    setLayout(processed_data_layout);
}

void ProcessedDataTab::addProcessedData()
{
    /* Get the type of component */
    ProcessedDataType add_type = ProcessedDataType(processed_data_combobox->currentIndex());
    QString type_str;

    if(add_type == DisparityImage)
        type_str = "Disparity Image";
    else if(add_type == Map)
        type_str = "Map";
    else if(add_type == Odometry)
        type_str = "Odometry";
    else if(add_type == ProcessedImage)
        type_str = "Processed Image";
    else
    {
        std::cerr << "ERROR -- Unknown Type encountered '" << add_type
                  << "' while adding processed data component" << std::endl;
        return;
    }

    if(add_type != Odometry) // Odometry needs a different dialog than ComponentDialog
    {
        ComponentDialog component_dialog(this);
        component_dialog.setWindowTitle(QString("Add %1").arg(type_str));

        if(component_dialog.exec())
        {
            QTreeWidgetItem *item = new QTreeWidgetItem;
            item->setText(0, type_str);
            item->setText(1, component_dialog.getName());
            item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
                (QStringList() << tr("Name") << component_dialog.getName())));
            item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
                (QStringList() << tr("Topic Name") << component_dialog.getTopicName())));

            processed_data_treewidget->addTopLevelItem(item);
        }
    }
    else
    {
        OdometryDialog odom_dialog(this);
        odom_dialog.setWindowTitle(QString("Add %1").arg(type_str));

        if(odom_dialog.exec())
        {
            QTreeWidgetItem *odom_item = new QTreeWidgetItem;
            odom_item->setText(0, type_str);
            odom_item->setText(1, odom_dialog.getName());
            odom_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
                (QStringList() << tr("Name") << odom_dialog.getName())));
            odom_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
                (QStringList() << tr("Topic Name") << odom_dialog.getTopicName())));

            // Set and add position state
            QString odom_pos("No");
            if(odom_dialog.isPositionChecked())
                odom_pos = "Yes";
            odom_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
                (QStringList() << tr("Position") << odom_pos)));

            // Set and add orientation state
            QString odom_ori("No");
            if(odom_dialog.isOrientationChecked())
                odom_ori = "Yes";
            odom_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
                (QStringList() << tr("Orientation") << odom_ori)));

            // Set and add linear velocity state
            QString odom_lin_vel("No");
            if(odom_dialog.isLinearVelocityChecked())
                odom_lin_vel = "Yes";
            odom_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
                (QStringList() << tr("Linear Velocity") << odom_lin_vel)));

            // Set and add angular velocity state
            QString odom_ang_vel("No");
            if(odom_dialog.isAngularVelocityChecked())
                odom_ang_vel = "Yes";
            odom_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
                (QStringList() << tr("Angular Velocity") << odom_ang_vel)));

            // Set and add attitude indicator state
            QString odom_show_attitude("No");
            if(odom_dialog.isShowAttitudeChecked())
                odom_show_attitude = "Yes";
            odom_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
                (QStringList() << tr("Show Attitude Indicator") << odom_show_attitude)));

            // Set and add heading indicator state
            QString odom_show_heading("No");
            if(odom_dialog.isShowHeadingChecked())
                odom_show_heading = "Yes";
            odom_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
                (QStringList() << tr("Show Heading Indicator") << odom_show_heading)));

            // Set and add labels state
            QString odom_show_labels("No");
            if(odom_dialog.isShowLabelsChecked())
                odom_show_labels = "Yes";
            odom_item->addChild(new QTreeWidgetItem((QTreeWidget *)0,
                (QStringList() << tr("Show Labels") << odom_show_labels)));

            processed_data_treewidget->addTopLevelItem(odom_item);
        }
    }
}

void ProcessedDataTab::editProcessedData()
{

}

///////////////////////// Joints Tab ////////////////////////////

JointsTab::JointsTab(struct RobotJoints *robot_joints, 
    QWidget *parent) : QWidget(parent)
{
    QLabel *topic_name_label = new QLabel(tr("Topic Name"));
    QLineEdit *topic_name_lineedit = new QLineEdit(robot_joints->topicName);

    QCheckBox *position_checkbox = new QCheckBox(tr("Position"));
    position_checkbox->setChecked(robot_joints->position);
    QCheckBox *velocity_checkbox = new QCheckBox(tr("Velocity"));
    velocity_checkbox->setChecked(robot_joints->velocity);
    QCheckBox *effort_checkbox = new QCheckBox(tr("Effort"));
    effort_checkbox->setChecked(robot_joints->effort);

    QLabel *joint_states_label = new QLabel(tr("Joints"));
    QPushButton *add_button = new QPushButton(tr("Add"));
    connect(add_button, SIGNAL(clicked()), this, SLOT(addJoint()));

    /* Add all joints from the robot configuration file */
    QList<QTreeWidgetItem *> joint_itemlist;
    for(unsigned int i = 0; i < robot_joints->joints.size(); i++)
        joint_itemlist.append(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << robot_joints->joints[i].name << robot_joints->joints[i].displayName)));

    QStringList column_names;
    column_names << "Joint Name" << "Display Name";

    /* Create tree widget */
    joints_treewidget = new QTreeWidget;
    joints_treewidget->setHeaderLabels(column_names);
    joints_treewidget->addTopLevelItems(joint_itemlist);
    joints_treewidget->setSortingEnabled(true);


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
    joints_layout->addWidget(joints_treewidget);
    setLayout(joints_layout);
}

void JointsTab::addJoint()
{
    ComponentDialog add_joint_dialog(this);
    add_joint_dialog.setNameLabelText(tr("Joint Name"));
    add_joint_dialog.setTopicNameLabelText(tr("Display Name"));
    add_joint_dialog.setWindowTitle("Add Joint");

    if(add_joint_dialog.exec())
    {
        joints_treewidget->addTopLevelItem(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << add_joint_dialog.getName() << add_joint_dialog.getTopicName())));
    }
}

void JointsTab::editJoint()
{

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

ServicesTab::ServicesTab(struct RobotCommands *robot_services, 
    QWidget *parent) : QWidget(parent)
{
    QLabel *services_label = new QLabel(tr("Services"));
    QStringList services_list;
    services_list << "Custom (Empty.srv)"
                  << "Land (Empty.srv)"
                  << "Takeoff (Empty.srv)";
    services_combobox = new QComboBox;
    services_combobox->addItems(services_list);
    QPushButton *add_button = new QPushButton(tr("Add"));
    connect(add_button, SIGNAL(clicked()), this, SLOT(addService()));

    
    /* Add all services from robot configuration file */
    QList<QTreeWidgetItem *> services_itemlist;
    for(unsigned int i = 0; i < robot_services->custom.size(); i++)
        services_itemlist.append(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << robot_services->custom[i].name << robot_services->custom[i].topicName)));

    QStringList column_list;
    column_list << "Service Name" << "Topic Name";

    /* Create tree widget */
    services_treewidget = new QTreeWidget;
    services_treewidget->setHeaderLabels(column_list);
    services_treewidget->addTopLevelItems(services_itemlist);

    /* Create services tab layout */
    QHBoxLayout *services_hlayout = new QHBoxLayout;
    services_hlayout->addWidget(services_label, Qt::AlignLeft);
    services_hlayout->addStretch();
    services_hlayout->addWidget(services_combobox, Qt::AlignRight);
    services_hlayout->addWidget(add_button, Qt::AlignRight);

    QVBoxLayout *services_layout = new QVBoxLayout;
    services_layout->addLayout(services_hlayout);
    services_layout->addWidget(services_treewidget);
    setLayout(services_layout);
}

void ServicesTab::addService()
{
    ComponentDialog add_service_dialog(this);
    add_service_dialog.setNameLabelText(tr("Service Name"));
    add_service_dialog.setWindowTitle("Add Service");

    if(services_combobox->currentText().startsWith(QString("Takeoff")))
        add_service_dialog.setName(tr("Takeoff"));
    else if(services_combobox->currentText().startsWith(QString("Land")))
        add_service_dialog.setName(tr("Land"));

    if(add_service_dialog.exec())
        services_treewidget->addTopLevelItem(new QTreeWidgetItem((QTreeWidget *)0,
            (QStringList() << add_service_dialog.getName() << add_service_dialog.getTopicName())));
}

void ServicesTab::editService()
{

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
RobotConfigFileDialog::RobotConfigFileDialog(
	struct RobotConfig *new_robot_config, QWidget *parent) : QDialog(parent)
{
	robot_config = new_robot_config;

    // Create tabs
    tab_widget = new QTabWidget;
    tab_widget->addTab(new GeneralTab(robot_config), tr("General"));
    tab_widget->addTab(new SensorsTab(robot_config), tr("Sensors"));
    tab_widget->addTab(new ProcessedDataTab(&robot_config->processedData), tr("Processed Data"));
    tab_widget->addTab(new JointsTab(&robot_config->joint_states), tr("Joints"));
    tab_widget->addTab(new ControlsTab, tr("Controls"));
    tab_widget->addTab(new ServicesTab(&robot_config->commands), tr("Services"));
    tab_widget->addTab(new DiagnosticsTab, tr("Diagnostics"));

    button_box = new QDialogButtonBox(QDialogButtonBox::Cancel |
                                      QDialogButtonBox::Save);
	connect(button_box, SIGNAL(accepted()), this, SLOT(accept()));
	connect(button_box, SIGNAL(rejected()), this, SLOT(reject()));

    // Create dialog layout
    QVBoxLayout *dialog_layout = new QVBoxLayout;
    dialog_layout->setSizeConstraint(QLayout::SetNoConstraint);
    dialog_layout->addWidget(tab_widget);
    dialog_layout->addWidget(button_box);
	setLayout(dialog_layout);
}
