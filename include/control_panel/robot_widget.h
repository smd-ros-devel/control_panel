/******************************************************************************
** robot_widget.h
**
** Author:      Matt Richard
** Date:        Oct 17, 2011
** Description: RobotWidget is the widget displayed in the scroll area in the
**              MainTab for one robot in the robot list. This reads every
**              robot's configuration file and displays the basic detailes of
**              the robot.
******************************************************************************/

#ifndef CONTROL_PANEL_ROBOT_WIDGET_H
#define CONTROL_PANEL_ROBOT_WIDGET_H

#include <QFrame>

QT_BEGIN_NAMESPACE
class QHBoxLayout;
class QLabel;
class QCheckBox;
class QImage;
class QPalette;
class QMouseEvent;
class QString;
QT_END_NAMESPACE

#include "robot_config.h"

#include <string>
#include <stdio.h>

class RobotWidget : public QFrame
{
	Q_OBJECT

	public:
		RobotWidget(QWidget *parent = 0);
		void setRobotPicture(const QImage &robot_image);
		void setRobotName(const std::string &name);
		QString getRobotName() const;
		QString getConfigPath() const;
		void setSystem(const std::string &robot_system);
		void setDriveSystem(const std::string &robot_drive_system);
		void setConfigPath(const std::string &robot_config_path);
		void setSelected(bool selected);
		bool isSelected() const;
		void setRobot(RobotConfig *rbt);

	signals:
		void singleRobotSelected(const QStringList &robot, bool auto_connect);

	protected:
		void mousePressEvent(QMouseEvent *event);
		void mouseDoubleClickEvent(QMouseEvent *event);

	private slots:
		void selectCheckboxChanged(int state);

	private:
		void createWidgets();
		void createLayout();

		QString robot_name;

		QString configFilePath;

		QHBoxLayout *robot_widget_layout;
		QPalette default_background_palette;
		QPalette selected_background_palette;

		QLabel *robot_picture_label;
		QLabel *robot_name_label;
		QLabel *system_label;
		QLabel *drive_system_label;

		QCheckBox *select_checkbox;
};

#endif // CONTROL_PANEL_ROBOT_WIDGET_H
