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
 * \file   robot_config_file_dialog.h
 * \date   Dec 6, 2011
 * \author Matt Richard
 */
#ifndef CONTROL_PANEL_ROBOT_CONFIG_FILE_DIALOG_H
#define CONTROL_PANEL_ROBOT_CONFIG_FILE_DIALOG_H

#include <QDialog>

QT_BEGIN_NAMESPACE
class QDialogButtonBox;
class QComboBox;
class QTabWidget;
class QTreeWidget;
QT_END_NAMESPACE

#include "component_dialogs.h"
#include "robot_config.h"

/**
 * \class GeneralTab
 * \brief
 */
class GeneralTab : public QWidget
{
    Q_OBJECT

    public:
        GeneralTab(struct RobotConfig *robot_config, QWidget *parent = 0);
};

/**
 * \class SensorsTab
 * \brief
 */
class SensorsTab : public QWidget
{
    Q_OBJECT

    public:
        SensorsTab(struct RobotSensors *robot_sensors, QWidget *parent = 0);

    public slots:
        void addSensor();
        void editSensor();
        void removeSensor();

    private:
        enum SensorType
        {
            Camera = 1001, // 1000 and below are reserved so start at 1001
            Compass,
            Gps,
            Imu,
            Laser,
            Range
        };

        QTreeWidget *sensors_treewidget;
        QComboBox *sensors_combobox;
};

/**
 * \class ProcessedDataTab
 * \brief
 */
class ProcessedDataTab : public QWidget
{
    Q_OBJECT

    public:
        ProcessedDataTab(struct RobotProcessedData *robot_processed_data,
                         QWidget *parent = 0);

    public slots:
        void addProcessedData();
        void editProcessedData();

    private:
        enum ProcessedDataType
        {
            DisparityImage = 1001,
            Map,
            Odometry,
            ProcessedImage
        };

        QTreeWidget *processed_data_treewidget;
        QComboBox *processed_data_combobox;
};

/**
 * \class JointsTab
 * \brief
 */
class JointsTab : public QWidget
{
    Q_OBJECT

    public:
        JointsTab(struct RobotJoints *robot_joints, QWidget *parent = 0);

    public slots:
        void addJoint();
        void editJoint();

    private:
        QTreeWidget *joints_treewidget;
};

/**
 * \class ControlsTab
 * \brief
 */
class ControlsTab : public QWidget
{
    Q_OBJECT

    public:
        ControlsTab(QWidget *parent = 0);
};

/**
 * \class ServicesTab
 * \brief
 */
class ServicesTab : public QWidget
{
    Q_OBJECT

    public:
        ServicesTab(struct RobotCommands *robot_services,
                    QWidget *parent = 0);

    public slots:
        void addService();
        void editService();

    private:
        QTreeWidget *services_treewidget;
        QComboBox *services_combobox;
};

/**
 * \class DiagnosticsTab
 * \brief
 */
class DiagnosticsTab : public QWidget
{
    Q_OBJECT

    public:
        DiagnosticsTab(QWidget *parent = 0);
};

/**
 * \class RobotConfigFileDialog
 * \brief Dialog for creating or editing a robot configuration file.
 */
class RobotConfigFileDialog : public QDialog
{
	Q_OBJECT

	public:
		RobotConfigFileDialog(struct RobotConfig *new_robot_config,
                              QWidget *parent = 0);

	private:
        QTabWidget *tab_widget;
        QDialogButtonBox *button_box;
		struct RobotConfig *robot_config;
};

#endif // CONTROL_PANEL_ROBOT_CONFIG_FILE_DIALOG_H
