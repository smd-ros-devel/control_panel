/* @todo Add license here. */

/**
 * \file   robot_config_file_dialog.h
 * \date   Dec 6, 2011
 * \author Matt Richard
 * Description: Dialog for creating or editing a robot configuration file.
 */
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
class QTabWidget;
QT_END_NAMESPACE

#include "add_sensor_dialog.h"
#include "robot_config.h"
#include "robot_config_sensor_widget.h"
#include <string>

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
        SensorsTab(struct RobotConfig *robot_config, QWidget *parent = 0);
};

/**
 * \class ProcessedDataTab
 * \brief
 */
class ProcessedDataTab : public QWidget
{
    Q_OBJECT

    public:
        ProcessedDataTab(QWidget *parent = 0);
};

/**
 * \class JointsTab
 * \brief
 */
class JointsTab : public QWidget
{
    Q_OBJECT

    public:
        JointsTab(QWidget *parent = 0);
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
        ServicesTab(QWidget *parent = 0);
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
		RobotConfigFileDialog(struct RobotConfig *new_robot_config, QWidget *parent = 0);
		void setTitle(const std::string &title);

	private slots:
		void addSensor();
		void editSelectedSensor();
		void removeSelectedSensors();

	private:
		void createWidgets();
		void createLayout();

        QTabWidget *tab_widget;
        QDialogButtonBox *button_box;

//		QGridLayout *dialog_layout;

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

//		QDialogButtonBox *save_cancel_buttonbox;

		struct RobotConfig *robot_config;

		RobotConfigSensorWidget *sensor_widget;
};

#endif // CONTROL_PANEL_ROBOT_CONFIG_FILE_DIALOG_H
