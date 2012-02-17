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
 * \file   main_tab.h
 * \date   Oct 10, 2011
 * \author Matt Richard, Scott K Logan
 *
 * The main tab displayed when the Control Panel is first
 * executed. This tab displays the list of known robots and allows
 * the user to select one or more robots to connect to and can
 * specify if the robot should be automatically connected to or
 * requires manual connection.
 */
#ifndef CONTROL_PANEL_MAIN_TAB_H
#define CONTROL_PANEL_MAIN_TAB_H

#include <QWidget>

QT_BEGIN_NAMESPACE
class QLabel;
class QPushButton;
class QCheckBox;
class QVBoxLayout;
class QScrollArea;
class QString;
class QStringList;
QT_END_NAMESPACE

#include <string>
#include "control_panel/robot_widget.h"


/**
 * \class MainTab
 * \brief
 */
class MainTab : public QWidget
{
	Q_OBJECT

	public:
		MainTab(const QString &robots, QWidget *parent = 0);

	signals:
		void loadRobots(const QStringList &robot_load_list, bool auto_connect);

	private slots:
		void deselectButtonClicked();
		void loadButtonPressed();

	private:
		void loadRobots();
		void createWidgets();
		void createLayout();

        QString robot_directory;

		QVBoxLayout *main_tab_layout;

		QLabel *robot_list_label;
        QLabel *ros_master_label;

		QScrollArea *robot_list_scrollarea;
		QWidget *robot_list_widget;
		QVBoxLayout *robot_list_layout;
		RobotWidget *robot_widget;

		QPushButton *deselect_all_button;
		QPushButton *load_button;

		QCheckBox *auto_connect_checkbox;
};

#endif // CONTROL_PANEL_MAIN_TAB_H
