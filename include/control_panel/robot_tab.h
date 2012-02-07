/******************************************************************************
 * robot_tab.h
 *
 * Author:      Matt Richard, Scott Logan
 * Date:        Aug 6, 2011
 * Description: Tab widget for a single robot. This displays the GUI and
 *              handles the ROS communication for a specified robot.
 *****************************************************************************/

#ifndef CONTROL_PANEL_ROBOT_TAB_H
#define CONTROL_PANEL_ROBOT_TAB_H

#include <QWidget>

QT_BEGIN_NAMESPACE
class QVBoxLayout;
QT_END_NAMESPACE

#include <stdio.h>
#include <string>

#include "data_pane.h"
#include "display_pane.h"
#include "node_manager.h"
#include "globals.h"
#include "robot_config.h"


class RobotTab : public QWidget
{
	Q_OBJECT

	public:
		RobotTab(struct RobotConfig *robot_config, QWidget *parent = 0);
		~RobotTab();
		void connectToRobot();
		void disconnectRobot();
		struct RobotConfig * getConfig();
		bool isKeyboardEnabled() const;
		bool robotConnected();
        void enableKeyboard();
        void disableKeyboard();
        void setRCMode(const QString &mode);

		NodeManager *node_manager;
        ControlNode *node; /* @todo Why is this here? */

	protected:
		void keyPressEvent(QKeyEvent *event);
		void keyReleaseEvent(QKeyEvent *event);

	signals:
		void connectionStatusChanged(int status, const QString &robot_name);

	private slots:
		void updateConnectionStatus(int status);
		void processDiagnostic(const QString &, const QString &);

	private:
		void createLayout();
        void setupDataPane();

		struct RobotConfig *robot_config;
		bool use_keyboard;

		QVBoxLayout *display_layout;

		// GUI Panes
		DataPane *data_pane;
        DisplayPane *raw_data_display;
        DisplayPane *processed_data_display;
};

#endif // CONTROL_PANEL_TAB_H
