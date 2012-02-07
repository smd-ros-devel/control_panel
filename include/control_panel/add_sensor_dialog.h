/******************************************************************************
 * add_sensor_dialog.h
 *
 * Author:      Matt Richard
 * Date:        Jan 9, 2012
 * Description:
 *****************************************************************************/

#ifndef CONTROL_PANEL_ADD_SENSOR_DIALOG_H
#define CONTROL_PANEL_ADD_SENSOR_DIALOG_H

#include <QDialog>

QT_BEGIN_NAMESPACE
class QDialogButtonBox;
class QCheckBox;
class QComboBox;
class QGridLayout;
class QLabel;
class QLineEdit;
class QPushButton;
QT_END_NAMESPACE

#include <QString>
#include "globals.h"

class AddSensorDialog : public QDialog
{
	Q_OBJECT

	public:
		AddSensorDialog(const QString &type, QWidget *parent = 0);
		QString getName() const;
		QString getTopicName() const;
		void setName(const QString &name);
		void setTopicName(const QString &topic_name);

		// GPS specific functions
		bool isLatChecked() const;
		bool isLongChecked() const;
		bool isAltChecked() const;
		bool isPosCovarChecked() const;

		// IMU specific functions
		bool isRollChecked() const;
		bool isPitchChecked() const;
		bool isYawChecked() const;
		bool isAngVelChecked() const;
		bool isAngVelCovarChecked() const;
		bool isLinAccelChecked() const;
		bool isLinAccelCovarChecked() const;

		// Temperature, velocity, and voltage specific functions
		QString getMin() const;
		QString getMax() const;

		// Temperature specific functions
		QString getUnits() const;

		// Voltage specific functions
		QString getVoltage() const;

	private:
		void createWidgets();
		void createLayout();

		QGridLayout *dialog_layout;
		QString sensor_type;

		// Generic for every robot
		QLabel *name_label;
		QLabel *topic_name_label;
		QLineEdit *name_lineedit;
		QLineEdit *topic_name_lineedit;

		QLabel *attributes_label;

		// GPS specific
		QCheckBox *lat_checkbox;       // Latitude
		QCheckBox *long_checkbox;      // Longitude
		QCheckBox *alt_checkbox;       // Altitude
		QCheckBox *pos_covar_checkbox; // Position Covariance

		// IMU specific
		QCheckBox *roll_checkbox;
		QCheckBox *pitch_checkbox;
		QCheckBox *yaw_checkbox;
		QCheckBox *ang_vel_checkbox;         // Angular Velocity
		QCheckBox *ang_vel_covar_checkbox;   // Angular Velocity Covariance
		QCheckBox *lin_accel_checkbox;       // Linear Acceleration
		QCheckBox *lin_accel_covar_checkbox; // Linear Acceleration Covariance

		// Temperature, velocity, and voltage specific
		QLabel *min_label;
		QLabel *max_label;
		QLineEdit *min_lineedit;
		QLineEdit *max_lineedit;

		// Temperature specific
		QLabel *units_label;
		QComboBox *units_combobox;

		// Voltage specific
		QLabel *voltage_label;
		QLineEdit *voltage_lineedit;

		QDialogButtonBox *cancel_add_buttonbox;
};

#endif // CONTROL_PANEL_ADD_SENSOR_DIALOG_H
