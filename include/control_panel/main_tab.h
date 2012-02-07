/******************************************************************************
** Author:      Matt Richard
** Date:        Oct. 10, 2011
** Description: The main tab displayed when the Control Panel is first
**              executed. This tab displays the list of known robots and allows
**              the user to select one or more robots to connect to and can
**              specify if the robot should be automatically connected to or
**              requires manual connection.
******************************************************************************/

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


class MainTab : public QWidget
{
	Q_OBJECT

	public:
		MainTab(const QString &robots, QWidget *parent = 0);
        void setMasterStatus(bool connected);

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
        QLabel *master_status_label;

		QScrollArea *robot_list_scrollarea;
		QWidget *robot_list_widget;
		QVBoxLayout *robot_list_layout;
		RobotWidget *robot_widget;

		QPushButton *deselect_all_button;
		QPushButton *load_button;

		QCheckBox *auto_connect_checkbox;
};

#endif // CONTROL_PANEL_MAIN_TAB_H
