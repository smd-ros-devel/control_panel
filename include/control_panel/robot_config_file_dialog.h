/******************************************************************************
** robot_config_file_dialog.h
**
** Author:      Matt Richard
** Date:        Dec 6, 2011
** Description: Dialog for creating or editing a robot configuration file.
******************************************************************************/

#ifndef CONTROL_PANEL_ROBOT_CONFIG_FILE_DIALOG_H
#define CONTROL_PANEL_ROBOT_CONFIG_FILE_DIALOG_H


#include <QDialog>

QT_BEGIN_NAMESPACE
class QDialogButtonBox;
class QComboBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QScrollArea;
class QGridLayout;
class QVBoxLayout;
QT_END_NAMESPACE

#include "add_sensor_dialog.h"
#include "robot_config.h"
#include "robot_config_sensor_widget.h"
#include <stdio.h>
#include <string>


class RobotConfigFileDialog : public QDialog
{
	Q_OBJECT

	public:
		RobotConfigFileDialog(struct RobotConfig *new_robot_config, QWidget *parent = 0);
		void setTitle(const std::string &title);

	private slots:
		void addSensor();
		void editSelectedSensor();
		void removeSelectedSensors();

	private:
		void createWidgets();
		void createLayout();

		QGridLayout *dialog_layout;

		QLabel *robot_name_label;
		QLabel *system_label;
		QLabel *drive_system_label;
		QLabel *image_file_label;
		QLabel *sensors_label;

		QLineEdit *robot_name_lineedit;
		QLineEdit *drive_system_lineedit;
		QLineEdit *image_file_lineedit;

		QComboBox *system_combobox;
		QComboBox *sensors_combobox;

		QPushButton *browse_button;
		QPushButton *add_button;
		QPushButton *edit_button;
		QPushButton *remove_button;

		QScrollArea *sensors_scrollarea;
		QVBoxLayout *sensor_list_layout;
		QWidget *sensor_list_widget;

		QDialogButtonBox *save_cancel_buttonbox;

		struct RobotConfig *robot_config;

		RobotConfigSensorWidget *sensor_widget;
};

#endif // CONTROL_PANEL_ROBOT_CONFIG_FILE_DIALOG_H
