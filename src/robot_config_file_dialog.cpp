/******************************************************************************
 * robot_config_file_dialog.cpp
 *
 * Author:      Matt Richard, Scott Logan
 * Date:        Dec 6, 2011
 * Description: Dialog for creating or editing a robot configuration file.
 *****************************************************************************/

#include <QtGui>
#include "control_panel/robot_config_file_dialog.h"

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

	createWidgets();
	createLayout();

	// Connections
	connect(save_cancel_buttonbox, SIGNAL(accepted()), this, SLOT(accept()));
	connect(save_cancel_buttonbox, SIGNAL(rejected()), this, SLOT(reject()));
	connect(add_button, SIGNAL(clicked()), this, SLOT(addSensor()));
	connect(edit_button, SIGNAL(clicked()), this, SLOT(editSelectedSensor()));
	connect(remove_button, SIGNAL(clicked()), this, SLOT(removeSelectedSensors()));

	setLayout(dialog_layout);
	layout()->setSizeConstraint(QLayout::SetFixedSize);
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
}


void RobotConfigFileDialog::setTitle(const std::string &title)
{
	setWindowTitle(title.c_str());
}

void RobotConfigFileDialog::addSensor()
{
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
}

void RobotConfigFileDialog::editSelectedSensor()
{

}

void RobotConfigFileDialog::removeSelectedSensors()
{

}
