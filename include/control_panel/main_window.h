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
 * \file   main_window.h
 * \date   June 2011
 * \author Matt Richard, Scott K Logan
 */
#ifndef CONTROL_PANEL_MAIN_WINDOW_H
#define CONTROL_PANEL_MAIN_WINDOW_H

#include <QMainWindow>

QT_BEGIN_NAMESPACE
class QAction;
class QActionGroup;
class QKeyEvent;
class QMenu;
class QMessageBox;
class QTabWidget;
class QString;
class QStringList;
QT_END_NAMESPACE

#include <QIcon>
#include <string>

#include "main_tab.h"
#include "robot_tab.h"
#include "robot_config_file_dialog.h"
#include "master_settings_dialog.h"
//#include "nodes/joystick_node.h"
//#include "nodes/qt_node.h"
#include "globals.h"


/**
 * \class MainWindow
 * \brief Handles the menus and tabs.
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT

    public:
        MainWindow(int argc, char **argv);
        void readSettings();
        void writeSettings();

    protected:
        void closeEvent(QCloseEvent *event);
		QMenu *createPopupMenu();
		void contextMenuEvent(QContextMenuEvent *event);

    private slots:
		void about();
        void closeCurrentTab(); // SLOT for keyboard shortcut to close tab action
		void closeTab(int index);
		void editRobotConfigFile();
        void editTopics();
        void setMaxVelocity();
        void editMasterSettings();
		void fullScreenChanged(bool checked);
		void help();
        void loadSelectedRobots(const QStringList &robot_list, bool auto_connect);
		void newRobotConfigFile();
		void openTabInWindow();
		//void openWidgetInWindow();
		//void updateJoystickAxis(int axis, double value);
		//void updateJoystickButton(int axis, bool state);
        void selectNextTab();
        void selectPreviousTab();
		void startConnection();
        void stopConnection();
		void tabChanged(int index);
		void toggleRC(QAction *action);
		void updateTabIcon(int status, const QString &robot_name);

    private:
        void createMenuActions();
        void createMenus();
		void createTab();

        QTabWidget *tab_widget;
		MainTab *main_tab;

        QString robot_directory;

		QIcon robot_disconnected_icon;
		QIcon robot_connecting_icon;
		QIcon robot_connected_icon;

        // Menu items
        QMenu *file_menu;
        QMenu *edit_menu;
		QMenu *view_menu;
        QMenu *connections_menu;
        QMenu *help_menu;

        // Menu action items
        QAction *new_robot_action;
        QAction *exit_action;
        QAction *master_settings_action;
		QAction *configuration_file_action;
        QAction *topics_action;
        QAction *set_velocity_action;
		QAction *gestures_action;
		QAction *show_menu_bar_action;
		QAction *full_screen_action;
		QAction *tab_in_window_action;
		QAction *widget_in_window_action;
		QAction *system_diagnostics_action;
        QAction *connect_action;
        QAction *disconnect_action;
		QAction *manual_mode_action;
		QAction *semiautonomous_mode_action;
		QAction *autonomous_mode_action;
        QAction *disable_rc_action;
		QAction *keyboard_rc_action;
		QAction *joystick_rc_action;
		QAction *help_action;
        QAction *about_action;

        // Keyboard shortcut actions
        QAction *next_tab_action;
        QAction *prev_tab_action;
        QAction *close_tab_action;


		QActionGroup *robot_mode_actiongroup;
		QActionGroup *robot_rc_actiongroup;

//		JoystickNode *joystick_node;

//        QtNode *qt_node;
};

#endif // CONTROL_PANEL_MAIN_WINDOW_H
