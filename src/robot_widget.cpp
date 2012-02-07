/******************************************************************************
** robot_widget.cpp
**
** Author:      Matt Richard
** Date:        Oct 17, 2011
** Description: RobotWidget is the widget displayed in the scroll area in the
**              MainTab for one robot in the robot list. This reads every
**              robot's configuration file and displays the basic detailes of
**              the robot.
******************************************************************************/

#include <QtGui>

#include "control_panel/robot_widget.h"

RobotWidget::RobotWidget(QWidget *parent) : QFrame(parent)
{
	createWidgets();
	createLayout();

	default_background_palette = palette();
	selected_background_palette.setColor(QPalette::Background,
		QColor(50, 150, 255, 25));

	setLayout(robot_widget_layout);
	setFixedHeight(100);
	setFrameStyle(QFrame::Box | QFrame::Sunken);
	setAutoFillBackground(true);
}

void RobotWidget::mousePressEvent(QMouseEvent *event)
{
	bool state = false;
	
	if(event->button() == Qt::LeftButton)
	{
		state = select_checkbox->isChecked();
		select_checkbox->setChecked(!state);
	}
}

void RobotWidget::mouseDoubleClickEvent(QMouseEvent *event)
{
	QStringList robot_list;
	robot_list << configFilePath;

	if(event->button() == Qt::LeftButton)
		emit singleRobotSelected(robot_list, true);

	setSelected(false);
}

void RobotWidget::createWidgets()
{
	robot_picture_label = new QLabel;
	robot_picture_label->setFixedSize(80, 80);

	QFont font;
	font.setPointSize(14);

	robot_name_label = new QLabel;
	robot_name_label->setFont(font);

	system_label = new QLabel(tr("System: "));

	drive_system_label = new QLabel(tr("Drive System: "));

	select_checkbox = new QCheckBox(tr("Select"));
	connect(select_checkbox, SIGNAL(stateChanged(int)),
		this, SLOT(selectCheckboxChanged(int)));
}

void RobotWidget::createLayout()
{
	QVBoxLayout *robot_name_vlayout = new QVBoxLayout;
	robot_name_vlayout->addWidget(robot_name_label, 0, Qt::AlignLeft);
	robot_name_vlayout->addWidget(system_label, 0, Qt::AlignLeft);
	robot_name_vlayout->addWidget(drive_system_label, 0, Qt::AlignLeft);

	robot_widget_layout = new QHBoxLayout;
	robot_widget_layout->addWidget(robot_picture_label, 0, Qt::AlignLeft);
	robot_widget_layout->addLayout(robot_name_vlayout);
	robot_widget_layout->addStretch();
	robot_widget_layout->addWidget(select_checkbox, 0, Qt::AlignRight);
}

void RobotWidget::setRobotPicture(const QImage &robot_image)
{
	robot_picture_label->setPixmap(QPixmap::fromImage(robot_image.scaled(
		robot_picture_label->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation)));
}

void RobotWidget::setRobotName(const std::string &name)
{
	robot_name = name.c_str();

	robot_name_label->setText(getRobotName());
}

QString RobotWidget::getRobotName() const
{
	if(robot_name.isEmpty())
		return QString("(unnamed robot)");
	return robot_name;
}

QString RobotWidget::getConfigPath() const
{
	return configFilePath;
}

void RobotWidget::setSystem(const std::string &robot_system)
{
	system_label->setText(tr("System: %1").arg(robot_system.c_str()));
}

void RobotWidget::setDriveSystem(const std::string &robot_drive_system)
{
	drive_system_label->setText(tr("Drive System: %1").arg(
		robot_drive_system.c_str()));
}

void RobotWidget::setConfigPath(const std::string &robot_config_path)
{
	configFilePath = robot_config_path.c_str();
}

void RobotWidget::setSelected(bool selected)
{
	select_checkbox->setChecked(selected);
}

bool RobotWidget::isSelected() const
{
	return select_checkbox->isChecked();
}

void RobotWidget::selectCheckboxChanged(int state)
{
	if(state == Qt::Checked)
		setPalette(selected_background_palette);
	else
		setPalette(default_background_palette);
}

void RobotWidget::setRobot(RobotConfig *rbt)
{
	setConfigPath(rbt->configFilePath.toStdString());
	setRobotName(rbt->robotName.toStdString());
	setSystem(rbt->system.toStdString());
	setDriveSystem(rbt->driveSystem.toStdString());
	setRobotPicture(rbt->image);
}

