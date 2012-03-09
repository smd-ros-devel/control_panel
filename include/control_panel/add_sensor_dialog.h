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
//class QLineEdit;
class QPushButton;
QT_END_NAMESPACE

#include <QLineEdit>
#include <QString>
#include "globals.h"

class AddSensorDialog : public QDialog
{
	Q_OBJECT

	public:
        enum SensorType
        {
            Camera,
            Compass,
            GPS,
            IMU,
            Laser,
            Range
        };

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

class AddProcessedDataDialog : public QDialog
{
    Q_OBJECT

    public:
};

/**
 * \class AddJointDialog
 * \brief Dialog for adding/editing a single robot joint.
 */
class JointDialog : public QDialog
{
    Q_OBJECT

    public:
        JointDialog(QWidget *parent = 0);
        JointDialog(const QString &joint_name, const QString &display_name,
                    QWidget *parent = 0);
        QString getDisplayName() const { return display_name_lineedit->text(); }
        QString getJointName() const { return joint_name_lineedit->text(); }
        void setDisplayName(const QString &name) { display_name_lineedit->setText(name); }
        void setJointName(const QString &name) { joint_name_lineedit->setText(name); }

    private:
        void createDialog();

        QLineEdit *joint_name_lineedit;
        QLineEdit *display_name_lineedit;

        QDialogButtonBox *button_box;
};

#endif // CONTROL_PANEL_ADD_SENSOR_DIALOG_H
