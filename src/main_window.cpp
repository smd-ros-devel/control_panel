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
 * \file   main_window.cpp
 * \date   June 2011
 * \author Matt Richard, Scott K Logan
 */
#include <QtGui>
#include "control_panel/main_window.h"
#include <stdio.h>


MainWindow::MainWindow(int argc, char **argv)
{
    qt_node = new QtNode(argc, argv, "control_panel");
    if(!qt_node->init())
    {
        exit(1);
    }

    // Load tab connection status icons
	robot_disconnected_icon.addFile(":/images/status_lights/status_light_grey.png");
	robot_connecting_icon.addFile(":/images/status_lights/status_light_yellow.png");
	robot_connected_icon.addFile(":/images/status_lights/status_light_green.png");


	// Create the menu
    createMenuActions();
    createMenus();

	// Create the Tab Widget
	tab_widget = new QTabWidget(this);
	tab_widget->setTabsClosable(true);

	// Create and insert the Main Tab
	main_tab = new MainTab(robot_directory);
	tab_widget->addTab(main_tab, "Main");


    // Keyboard shortcut actions
    close_tab_action = new QAction(tab_widget);
    close_tab_action->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_W));
    //close_tab_action->setShortcutContext(Qt::ApplicationShortcut);
    connect(close_tab_action, SIGNAL(triggered()), this, SLOT(closeCurrentTab()));


	// Connections
	connect(main_tab, SIGNAL(loadRobots(const QStringList &, bool)),
		this, SLOT(loadSelectedRobots(const QStringList &, bool)));
	connect(tab_widget, SIGNAL(currentChanged(int)),
		this, SLOT(tabChanged(int)));
	connect(tab_widget, SIGNAL(tabCloseRequested(int)),
		this, SLOT(closeTab(int)));

	// Main window settings
    readSettings();
	setCentralWidget(tab_widget);
    setWindowTitle("SRS Control Panel");

}

void MainWindow::readSettings()
{
    printf("Reading settings\n");

    QSettings settings;
    if(settings.value("maximized", false).toBool())
        showMaximized();
}

void MainWindow::writeSettings()
{
    printf("Saving settings\n");

    QSettings settings;
    settings.setValue("maximized", isMaximized());
}

/******************************************************************************
 * Function:    closeEvent
 * Author:      Matt Richard
 * Parameters:  QCloseEvent *event
 * Returns:     void
 * Description: For now, just accepts the close event.
 *****************************************************************************/
void MainWindow::closeEvent(QCloseEvent *event)
{
    int button_pushed;
    RobotTab *tab;
    bool robot_connected = false;

    // Check if any robots are currently connected
    for(int i = 1; i < tab_widget->count() && !robot_connected; i++)
    {
        tab = (RobotTab *)tab_widget->widget(i);

        if(tab->robotConnected())
            robot_connected = true;
    }

    // Verify the user wants to exit the program if there is a robot connect
    if(robot_connected)
    {
        button_pushed = QMessageBox::warning(this, tr("Disconnect warning"),
            tr("One or more robots are currently connected.\n") +
            tr("Disconnect from robot(s) and close Control Panel?"),
            QMessageBox::Yes | QMessageBox::No, QMessageBox::No);

        if(button_pushed != QMessageBox::Yes)
        {
            event->ignore();
            return;
        }
    }


	printf("Closing SRS Control Panel\n");
    printf("Disconnecting from all robots\n");

    // Disconnect from all connected robots
    while(tab_widget->count() > 1)
    {
        tab = (RobotTab *)tab_widget->widget(1);

        if(tab->robotConnected())
            tab->disconnectRobot();

        tab_widget->removeTab(1);
        delete tab;
    }

    writeSettings();

    event->accept();
}

/******************************************************************************
 * Function:    contextMenuEvent
 * Author:      Matt Richard
 * Parameters:  QContextMenuEvent *event
 * Returns:     void
 * Description: This is an overloaded function from QMainWindow. It creates
 *              the menu when the user right clicks inside the window.
 *****************************************************************************/
void MainWindow::contextMenuEvent(QContextMenuEvent *event)
{
	QMenu *menu = createPopupMenu();

	menu->exec(event->globalPos());
}

/******************************************************************************
 * Function:
 * Author:
 * Parameters:
 * Returns:
 * Description: This is an overloaded function from QMainWindow. It populates
 *              a QMenu with the items to be displayed when the user right
 *              clicks inside the window.
 *****************************************************************************/
QMenu *MainWindow::createPopupMenu()
{
	QMenu *popup_menu = new QMenu;

	popup_menu->addAction(widget_in_window_action);
	popup_menu->addAction(tab_in_window_action);
	popup_menu->addSeparator();
	popup_menu->addAction(show_menu_bar_action);
	popup_menu->addAction(full_screen_action);

	return popup_menu;
}

/******************************************************************************
 * Function:    createMenuActions
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     void
 * Description: Creates the actions for each menu item and connects the
 *              menu actions' signals to appropriate slots.
 *****************************************************************************/
void MainWindow::createMenuActions()
{
	/**
	** File Menu Actions
	**/
	new_robot_action = new QAction(tr("&New Robot Configuration File"), this);
	new_robot_action->setShortcut(QKeySequence::New);
	connect(new_robot_action, SIGNAL(triggered()),
		this, SLOT(newRobotConfigFile()));

    exit_action = new QAction(tr("E&xit"), this);
    exit_action->setShortcuts(QKeySequence::Quit);
    connect(exit_action, SIGNAL(triggered()), this, SLOT(close()));


	/**
	** Edit Menu Actions
	**/
    master_settings_action = new QAction(tr("ROS &Master Settings"), this);
    master_settings_action->setEnabled(false);
    connect(master_settings_action, SIGNAL(triggered()), SLOT(editMasterSettings()));

	configuration_file_action = new QAction(tr("Robot Configuration &File"),
		this);
	configuration_file_action->setEnabled(false);
	connect(configuration_file_action, SIGNAL(triggered()),
		this, SLOT(editRobotConfigFile()));

    topics_action = new QAction(tr("&Topic Names"), this);
	topics_action->setEnabled(false);
    //connect(topics_action, SIGNAL(triggered()), this, SLOT(editTopicNames()));

    set_velocity_action = new QAction(tr("Set Velocity Scale"), this);
    set_velocity_action->setEnabled(false);
    set_velocity_action->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_S));
    connect(set_velocity_action, SIGNAL(triggered()), SLOT(setMaxVelocity()));

	gestures_action = new QAction(tr("Kinect &Gestures"), this);
	gestures_action->setEnabled(false);


	/**
	** View Menu Actions
	**/
	show_menu_bar_action = new QAction(tr("Show &Menu Bar"), this);
	show_menu_bar_action->setCheckable(true);
	show_menu_bar_action->setChecked(true);
	connect(show_menu_bar_action, SIGNAL(toggled(bool)),
		menuBar(), SLOT(setVisible(bool)));

	full_screen_action = new QAction(tr("&Full Screen"), this);
	full_screen_action->setShortcut(Qt::Key_F11);
	full_screen_action->setCheckable(true);
    //full_screen_action->setShortcutContext(Qt::ApplicationShortcut);
	connect(full_screen_action, SIGNAL(toggled(bool)), 
		this, SLOT(fullScreenChanged(bool)));

	widget_in_window_action = new QAction(tr("Open Widget in New Window"), 
		this);
	widget_in_window_action->setEnabled(false);
	//connect(widget_in_window_action, SIGNAL(triggered()), this, SLOT(openWidgetInWindow()));

	tab_in_window_action = new QAction(tr("Open Tab in New Window"), this);
	tab_in_window_action->setEnabled(false);
	//connect(tab_in_window_action, SIGNAL(triggered()), this, SLOT(openTabInWindow()));

	system_diagnostics_action = new QAction(tr("Open System Diagnostics Tab"), 
		this);
	system_diagnostics_action->setEnabled(false);


	/**
	** Connections Menu Actions
	**/
    connect_action = new QAction(tr("&Connect"), this);
	connect_action->setEnabled(false);
	connect_action->setShortcut(QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_C));
	connect(connect_action, SIGNAL(triggered()),
		this, SLOT(startConnection()));

    disconnect_action = new QAction(tr("&Disconnect"), this);
    disconnect_action->setEnabled(false);
	disconnect_action->setShortcut(QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_D));
    connect(disconnect_action, SIGNAL(triggered()),
		this, SLOT(stopConnection()));


	manual_mode_action = new QAction(tr("Manual"), this);
	manual_mode_action->setCheckable(true);

	semiautonomous_mode_action = new QAction(tr("Semi-autonomous"), this);
	semiautonomous_mode_action->setCheckable(true);

	autonomous_mode_action = new QAction(tr("Autonomous"), this);
	autonomous_mode_action->setCheckable(true);

	robot_mode_actiongroup = new QActionGroup(this);
	robot_mode_actiongroup->addAction(manual_mode_action);
	robot_mode_actiongroup->addAction(semiautonomous_mode_action);
	robot_mode_actiongroup->addAction(autonomous_mode_action);
	manual_mode_action->setChecked(true);
	robot_mode_actiongroup->setEnabled(false);


    disable_rc_action = new QAction(tr("&Disable RC"), this);
    disable_rc_action->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_R));
    disable_rc_action->setCheckable(true);

	keyboard_rc_action = new QAction(tr("&Keyboard RC"), this);
    keyboard_rc_action->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_K));
	keyboard_rc_action->setCheckable(true);

	joystick_rc_action = new QAction(tr("&Joystick RC"), this);
    joystick_rc_action->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_J));
	joystick_rc_action->setCheckable(true);

	robot_rc_actiongroup = new QActionGroup(this);
    robot_rc_actiongroup->addAction(disable_rc_action);
	robot_rc_actiongroup->addAction(keyboard_rc_action);
	robot_rc_actiongroup->addAction(joystick_rc_action);
    disable_rc_action->setChecked(true);
	keyboard_rc_action->setChecked(false);
	robot_rc_actiongroup->setEnabled(false);
	connect(robot_rc_actiongroup, SIGNAL(triggered(QAction *)),
		this, SLOT(toggleRC(QAction *)));


    /* Tools Menu Actions */
    call_service_action = new QAction(tr("Call Service"), this);
    connect(call_service_action, SIGNAL(triggered()), this, SLOT(callService()));

    rxconsole_action = new QAction(tr("Rx&console"), this);
    connect(rxconsole_action, SIGNAL(triggered()), this, SLOT(startRxconsole()));

    rxgraph_action = new QAction(tr("Rx&graph"), this);
    connect(rxgraph_action, SIGNAL(triggered()), this, SLOT(startRxgraph()));

    dynamic_reconfigure_action = new QAction(tr("Dynamic &Reconfigure"), this);
    connect(dynamic_reconfigure_action, SIGNAL(triggered()), this, SLOT(startDynamicReconfigure()));


	/* Help Menu Actions */
	help_action = new QAction(tr("&Help"), this);
	help_action->setShortcuts(QKeySequence::HelpContents);
	help_action->setEnabled(false);
	connect(help_action, SIGNAL(triggered()), this, SLOT(help()));

    about_action = new QAction(tr("&About SRS Control Panel"), this);
    connect(about_action, SIGNAL(triggered()), this, SLOT(about()));
}

/******************************************************************************
 * Function:    createMenus
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     void
 * Description: Creates each menu.
 *****************************************************************************/
void MainWindow::createMenus()
{
	// Create file menu
    file_menu = menuBar()->addMenu(tr("&File"));
	file_menu->addAction(new_robot_action);
	file_menu->addSeparator();
    file_menu->addAction(exit_action);

	// Create edit menu
    edit_menu = menuBar()->addMenu(tr("&Edit"));
    //edit_menu->addAction(master_settings_action);
	edit_menu->addAction(configuration_file_action);
    //edit_menu->addAction(topics_action);
    edit_menu->addAction(set_velocity_action);
	//edit_menu->addSeparator();
	//edit_menu->addAction(gestures_action);

	// Create view menu
	view_menu = menuBar()->addMenu(tr("&View"));
	view_menu->addAction(show_menu_bar_action);
	view_menu->addAction(full_screen_action);
	view_menu->addSeparator();
	//view_menu->addAction(tab_in_window_action);
	//view_menu->addAction(system_diagnostics_action);

	// Create connections menu
    connections_menu = menuBar()->addMenu(tr("&Connections"));
	connections_menu->addAction(connect_action);
    connections_menu->addAction(disconnect_action);
	//connections_menu->addSeparator()->setText(tr("Robot Mode"));
	//connections_menu->addAction(manual_mode_action);
	//connections_menu->addAction(semiautonomous_mode_action);
	//connections_menu->addAction(autonomous_mode_action);
	connections_menu->addSeparator()->setText(tr("RC Mode"));
    connections_menu->addAction(disable_rc_action);
	connections_menu->addAction(keyboard_rc_action);
	connections_menu->addAction(joystick_rc_action);

    // Create tools menu
    tools_menu = menuBar()->addMenu(tr("&Tools"));
    tools_menu->addAction(call_service_action);
    tools_menu->addAction(rxconsole_action);
    tools_menu->addAction(rxgraph_action);
    tools_menu->addAction(dynamic_reconfigure_action);

	// Create help menu
    help_menu = menuBar()->addMenu(tr("&Help"));
	help_menu->addAction(help_action);
	help_menu->addSeparator();
    help_menu->addAction(about_action);
}

void MainWindow::editMasterSettings()
{
/*
    MasterSettingsDialog master_settings;
    master_settings.setMasterURI(QString(qt_node->getMasterURI().c_str()));
    master_settings.setHostIP(QString(qt_node->getHostIP().c_str()));

    if(master_settings.exec())
    {

        qt_node->stop();

        if(!master_settings.disconnectPushed())
        {

            bool use_env_vars = master_settings.useEnvironmentalVariables();
            if(!use_env_vars)
            {
                qt_node->setMasterURI(master_settings.getMasterURI());
                qt_node->setHostIP(master_settings.getHostIP());
            }

            bool success = qt_node->init(use_env_vars);

            main_tab->setMasterStatus(success);
        }
        else
            main_tab->setMasterStatus(false);

    }
*/
}

/******************************************************************************
 * Function:    editRobotConfigFile
 * Author:      Matt Richard, Scott Logan
 * Parameters:  None
 * Returns:     void
 * Description:
 *****************************************************************************/
void MainWindow::editRobotConfigFile()
{
	// Get the index of the current tab
	int curr_index = tab_widget->currentIndex();
	RobotTab *tab;

    // Check if the index in the Main Tab.
	if(curr_index != 0)
	{
		tab = (RobotTab *)tab_widget->currentWidget();

		RobotConfigFileDialog edit_robot_dialog(tab->getConfig());
		edit_robot_dialog.setTitle("Edit Robot Configuration File");

		if(edit_robot_dialog.exec())
		{

		}
	}
}

/******************************************************************************
 * Function:    editTopics
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     void
 * Description:
 *****************************************************************************/
void MainWindow::editTopics()
{
	// @todo
	// Create a dialog allowing the user to edit the topic names over which
	// each node is publishing or subscribing to for a connected robot.
}

void MainWindow::setMaxVelocity()
{
    bool ok;
    double scale;
    RobotTab *tab;

    // Return if tab is the main tab
    if(tab_widget->currentIndex() == 0)
        return;

    tab = (RobotTab *)tab_widget->currentWidget();

    // Make sure the robot has a control node
    if(tab->node_manager->control_node)
    {
        // Prompt use for new scale
        scale = QInputDialog::getDouble(this, tr("Set Velocity Scale"),
            tr("Scale:"), tab->node_manager->control_node->getScale(),
            0.00, 1.00, 2, &ok);

        // Set scale
        if(ok)
            tab->node_manager->control_node->setScale(scale);
    }
}

/******************************************************************************
 * Function:    help
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     void
 * Description: 
 *****************************************************************************/
void MainWindow::help()
{
	// This will start the help documentation.

	// @todo
	// Create help documentation.
}

/******************************************************************************
 * Function:    about
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     void
 * Description: Displays a message box containing brief information about the
 *              SRS Control Panel.
 *****************************************************************************/
void MainWindow::about()
{
	// @todo
	// Finish Control Panel About information
	QMessageBox::about(this, tr("About SRS Control Panel"),
		tr("<h1>Shared Robotics Systems<br/>") +
		tr("Control Panel</h1>") +
        tr("<b>Version 4.1</b>") +
        tr("<p>TODO: Add Control Panel information here.</p>"));
}

/******************************************************************************
 * Function:    loadSelectedRobots
 * Author:      Matt Richard, Scott Logan
 * Parameters:  const QStringList &robot_list - List of robots names that were
 *                  selected to be loaded.
 *              bool auto_connect - If true, the thread to 
 * Returns:     void
 * Description:
 *****************************************************************************/
void MainWindow::loadSelectedRobots(const QStringList &robot_list,
	bool auto_connect)
{
	int index_inserted = 0;
	int i, j;
	bool found = false;
	RobotTab *tab;
	RobotConfig *robot_config;

	// Loop through each robot in the list of selected robots
	for(i = 0; i < robot_list.count(); i++)
	{
		found = false;

		QFile curr_file(robot_list.at(i));

		std::cout << "Loading full robot configuration from " 
				  << curr_file.fileName().toStdString() << std::endl;
		robot_config = new RobotConfig;

		if(curr_file.open(QIODevice::ReadOnly) && 
			!robot_config->loadFrom(&curr_file, true))
		{
			// Check if the selected robot has already been loaded
			for(j = 1; j < tab_widget->count() && !found; j++)
				if(robot_config->getRobotName() == tab_widget->tabText(j))
					found = true;

			if(!found)
			{
				// Create a tab for the selected robot
				tab = new RobotTab(robot_config);

				// Insert tab
				index_inserted = tab_widget->addTab(tab, robot_disconnected_icon,
					robot_config->getRobotName());

                connect(tab, SIGNAL(connectionStatusChanged(int, const QString &)),
                    this, SLOT(updateTabIcon(int, const QString &)));

				// Connect to robot if specified by user
				if(auto_connect)
					tab->connectToRobot();
			}
			else // Selected robot is already loaded.
				index_inserted = j - 1;
		}
		else
		{
			std::cerr << "Error while loading robot configuration from "
					  << curr_file.fileName().toStdString() << std::endl;
		}

		curr_file.close();
	}

	// Set the current tab to the last robot loaded
	tab_widget->setCurrentIndex(index_inserted);
}

/******************************************************************************
 * Function:    startConnection
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     void
 * Description: Starts the connection between the Control Panel and the
 *              currently selected robot.
 *****************************************************************************/
void MainWindow::startConnection()
{
	// Get the index of the current tab.
	int curr_index = tab_widget->currentIndex();
	RobotTab *curr_tab;

    // Check if the current tab is the Main Tab.
	if(curr_index != 0)
	{
		curr_tab = (RobotTab *)tab_widget->currentWidget();

		// Connect tot he robot if not already connected
		if(!curr_tab->robotConnected())
		{
			curr_tab->connectToRobot();

			connect_action->setEnabled(false);
			disconnect_action->setEnabled(true);
		}
	}
}

/******************************************************************************
 * Function:    stopConnection
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     void
 * Description: Stops the connection between the Control Panel and the
 *              currently selected robot.
 *****************************************************************************/
void MainWindow::stopConnection()
{
	// Get the index of the current tab.
	int curr_index = tab_widget->currentIndex();
	RobotTab *curr_tab;

	// Make sure the current tab is not the main tab
	if(curr_index != 0)
	{
		curr_tab = (RobotTab *)tab_widget->currentWidget();

		// Disconnect from the robot if not already
		if(curr_tab->robotConnected())
		{
			curr_tab->disconnectRobot();

			disconnect_action->setEnabled(false);
			connect_action->setEnabled(true);
		}
	}
}

/******************************************************************************
 * Function:    closeTab
 * Author:      Matt Richard
 * Parameters:  int index - index of tab requested to close
 * Returns:     void
 * Description: Disconnects from the robot if the Control Panel is currently
 *              connected to it, then closes the tab.
 *****************************************************************************/
void MainWindow::closeTab(int index)
{
    int button_pushed;
	RobotTab *tab;

	// Check if the tab is main tab or not
	if(index != 0)
	{
		tab = (RobotTab *)tab_widget->widget(index);

		// Check if the robot is currently connected
		if(tab->robotConnected())
		{
			// Verify the user wants to close the tab
			button_pushed = QMessageBox::warning(this, tr("Disconnect warning"),
				tr("Are you sure you want to disconnect from this robot?"),
				QMessageBox::Yes | QMessageBox::No, QMessageBox::No);

			// Return if the user doesn't push yes.
			if(button_pushed != QMessageBox::Yes)
				return;

			tab->disconnectRobot();
		}

		// Remove the tab and free memory.
		tab_widget->removeTab(index);
		delete tab;
	}
}

/******************************************************************************
 * Function:    fullScreenChanged
 * Author:      Matt Richard
 * Parameters:  bool checked - true if fullscreen menu action is checked,
 *                  otherwise false.
 * Returns:     void
 * Description: Toggles between fullscreen and normal.
 *****************************************************************************/
void MainWindow::fullScreenChanged(bool checked)
{
	if(checked)
		showFullScreen();
	else
		showNormal();
}

/******************************************************************************
 * Function:    openTabInWindow
 * Author:      
 * Parameters:
 * Returns:     void
 * Description:
 *****************************************************************************/
void MainWindow::openTabInWindow()
{
}

/******************************************************************************
 * Function:    newRobotConfigFile
 * Author:      Matt Richard, Scott Logan
 * Parameters:  None
 * Returns:     void
 * Description: 
 *****************************************************************************/
void MainWindow::newRobotConfigFile()
{
	/**
	 * TODO:
	 * Where do we put the new configuration?
	 **/
	RobotConfig *new_robot_config = new RobotConfig;

	RobotConfigFileDialog new_robot_dialog(new_robot_config);
	new_robot_dialog.setTitle("New Robot Configuration File");

	if(new_robot_dialog.exec())
	{

	}
}

/******************************************************************************
 * Function:    updateJoystickAxis
 * Author:      Matt Richard
 * Parameters:  int axis - which axis changed
 *              double value - the new value for that axis
 * Returns:     void
 * Description:
 *****************************************************************************/
/*
void MainWindow::updateJoystickAxis(int axis, double value)
{
	RobotTab *tab;

	// Check if the current tab is the Main Tab.
	if(tab_widget->currentIndex() != 0)
	{
		tab = (RobotTab *)tab_widget->currentWidget();

		// Verify the robot is connected before publishing message
		if(tab->robotConnected())
		{
			// Joystick mapping here
			if(axis == 0 && tab->node_manager->control_node)
				tab->node_manager->control_node->setLinearY(value);
			else if(axis == 1 && tab->node_manager->control_node)
				tab->node_manager->control_node->setLinearX(value);
			else if(axis == 2 && tab->node_manager->control_node)
				tab->node_manager->control_node->setAngularZ(value);
			else if(axis == 3 && tab->node_manager->control_node)
				tab->node_manager->control_node->setLinearZ(value);
		}
	}
}
*/

/******************************************************************************
 * Function:    updateJoystickButton
 * Author:      Scott K Logan
 * Parameters:  int button - which button changed
 *              double state - state of the button in question
 * Returns:     void
 * Description:
 *****************************************************************************/
/*
void MainWindow::updateJoystickButton(int button, bool state)
{
	RobotTab *tab;

	// Check if the current tab is the Main Tab.
	if(tab_widget->currentIndex() != 0)
	{
		tab = (RobotTab *)tab_widget->currentWidget();

		// Verify the robot is connected before publishing message
		// Also, only fire on key PRESS event (state == true)
		if(tab->robotConnected() && state)
		{
			// Joystick mapping here
			if(button == 14)
			{
				// Takeoff Message
				struct RobotCommandCustom *custom_temp = tab->getConfig()->commands.custom;
				while(custom_temp != NULL)
				{
					if(custom_temp->name == "takeoff")
						tab->node_manager->command_node->callEmpty(custom_temp->topicName);
					custom_temp = custom_temp->next;
				}
			}
			if(button == 13)
			{
				// Land Message
				struct RobotCommandCustom *custom_temp = tab->getConfig()->commands.custom;
				while(custom_temp != NULL)
				{
					if(custom_temp->name == "land")
						tab->node_manager->command_node->callEmpty(custom_temp->topicName);
					custom_temp = custom_temp->next;
				}
			}
			if(button == 12)
			{
				// Reset Message
				struct RobotCommandCustom *custom_temp = tab->getConfig()->commands.custom;
				while(custom_temp != NULL)
				{
					if(custom_temp->name == "Reset")
						tab->node_manager->command_node->callEmpty(custom_temp->topicName);
					custom_temp = custom_temp->next;
				}
			}
			if(button == 15)
			{
				// Land Message
				struct RobotCommandCustom *custom_temp = tab->getConfig()->commands.custom;
				while(custom_temp != NULL)
				{
					if(custom_temp->name == "Camera Toggle")
						tab->node_manager->command_node->callEmpty(custom_temp->topicName);
					custom_temp = custom_temp->next;
				}
			}
		}
	}
}
*/

/******************************************************************************
 * Function:    tabChanged
 * Author:      Matt Richard
 * Parameters:  int index - index of the now current tab.
 * Returns:     void
 * Description:
 *****************************************************************************/
void MainWindow::tabChanged(int index)
{
	RobotTab *tab;

	connect_action->setEnabled(false);
	disconnect_action->setEnabled(false);
	configuration_file_action->setEnabled(false);
	robot_rc_actiongroup->setEnabled(false);

	// Check if the current tab is the Main Tab.
	if(index != 0)
	{
		tab = (RobotTab *)tab_widget->currentWidget();

		if(tab->robotConnected())
			disconnect_action->setEnabled(true);
		else
			connect_action->setEnabled(true);

		configuration_file_action->setEnabled(true);
		robot_rc_actiongroup->setEnabled(true);

        if(tab->node_manager->control_node)
            set_velocity_action->setEnabled(true);
        else
            set_velocity_action->setEnabled(false);
	}
}

/******************************************************************************
 * Function:    toggleRC
 * Author:      Matt Richard
 * Parameters:  QAction * - The action triggered.
 * Returns:     void
 * Description: Toggles robot remote control mode.
 *****************************************************************************/
void MainWindow::toggleRC(QAction *action)
{
	RobotTab *tab;
    QString mode;

	// Check if current tab is not the Main Tab.
	if(tab_widget->currentIndex() != 0)
	{
		// Get current tab widget
		tab = (RobotTab *)tab_widget->currentWidget();

        if(action == disable_rc_action)
            tab->setRCMode(Globals::Disabled);
		else if(action == keyboard_rc_action)
            tab->setRCMode(Globals::Keyboard);
	    else if(action == joystick_rc_action)
            tab->setRCMode(Globals::Joystick);
        else
            printf("ERROR -- Unknown RC action\n");
    }
}

/******************************************************************************
 * Function:    updateTabIcon
 * Author:      Matt Richard
 * Parameters:  int status - the connection status with the robot
 *              const QString &robot_name - robot's name that changed status
 * Returns:     void
 * Description: 
 *****************************************************************************/
void MainWindow::updateTabIcon(int status, const QString &robot_name)
{
	int i = 0;

	// Find the position of the tab to update
	while(i <= tab_widget->count() && robot_name != tab_widget->tabText(i))
		i++;

	// Check if the tab wasn't found
	if(i > tab_widget->count())
	{
		printf("ERROR -- robot tab '%s' could not be found\n",
			robot_name.toStdString().c_str());
		return;
	}

	// Load status light icon
	if(status == Globals::Disconnected)
		tab_widget->setTabIcon(i, robot_disconnected_icon);
    else if(status == Globals::Connecting)
		tab_widget->setTabIcon(i, robot_connecting_icon);
	else if(status == Globals::Connected)
		tab_widget->setTabIcon(i, robot_connected_icon);
	else
		printf("ERROR -- unknown status detected when updating robot tab's icon\n");
}

void MainWindow::closeCurrentTab()
{
    closeTab(tab_widget->currentIndex());
}

void MainWindow::callService()
{
    bool ok;

    QString service_name = QInputDialog::getText(this, tr("Call Service"),
        tr("Service Name"), QLineEdit::Normal, tr(""), &ok);

    if(ok && !service_name.isEmpty())
        if(qt_node->service_node)
            qt_node->service_node->callEmpty(service_name);
}

void MainWindow::startRxconsole()
{
    if(QProcess::startDetached("rxconsole"))
        printf("rxconsole started successfully\n");
    else
    {
        perror("rxconsole failed to start. It may not be installed.\n");
        /* @todo Warn user of failure */
    }
}

void MainWindow::startRxgraph()
{
    if(QProcess::startDetached("rxgraph"))
        printf("rxgraph started successfully\n");
    else
    {
        perror("rxgraph failed to start. It may not be installed.\n");
        /* @todo Display a QMessageBox warning the user that rxgraph was not executed successfully */
    }
}

void MainWindow::startDynamicReconfigure()
{
    if(QProcess::startDetached("rosrun dynamic_reconfigure reconfigure_gui"))
        printf("reconfigure_gui started successfully\n");
    else
    {
        perror("reconfigure_gui failed to start. It may not be installed.\n");
        /* @todo Display a QMessageBox warning the user that reconfigure_gui was not executed successfully */
    }
}
