/******************************************************************************
 * add_sensor_dialog.cpp
 *
 * Author:      Matt Richard
 * Date:        Jan 9, 2012
 * Description:
 *****************************************************************************/

#include <QtGui>
#include "control_panel/add_sensor_dialog.h"

/******************************************************************************
 * Function:    AddSensorDialog
 * Author:      Matt Richard
 * Parameters:  const QString &type - the type of sensor that will be added
 *              QWidget *parent - the parent widget
 * Returns:     None
 * Description: Constructor.
 *****************************************************************************/
AddSensorDialog::AddSensorDialog(const QString &type, QWidget *parent)
	: QDialog(parent)
{
	sensor_type = type;

	createWidgets();
	createLayout();

	// Connections:
	connect(cancel_add_buttonbox, SIGNAL(accepted()), this, SLOT(accept()));
	connect(cancel_add_buttonbox, SIGNAL(rejected()), this, SLOT(reject()));

	// Dialog settings
	setLayout(dialog_layout);
	layout()->setSizeConstraint(QLayout::SetFixedSize);
	setWindowTitle(tr("Add ") + sensor_type);
}

/******************************************************************************
 * Function:    createWidgets
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     void
 * Description: Initializes all widgets to be used by this dialog.
 *****************************************************************************/
void AddSensorDialog::createWidgets()
{
	name_label = new QLabel(tr("Name"));
	name_lineedit = new QLineEdit;

	topic_name_label = new QLabel(tr("Topic Name"));
	topic_name_lineedit = new QLineEdit;


	if(sensor_type == "GPS" || sensor_type == "IMU")
		attributes_label = new QLabel("Attributes");


	QString topic_name;
	if(sensor_type == "Camera")
		topic_name = Globals::DEFAULT_CAMERA_TOPIC.c_str();
	else if(sensor_type == "GPS")
	{
		topic_name = Globals::DEFAULT_GPS_TOPIC.c_str();

		lat_checkbox = new QCheckBox("Latitude");
		lat_checkbox->setChecked(true);

		long_checkbox = new QCheckBox("Longitude");
		long_checkbox->setChecked(true);

		alt_checkbox = new QCheckBox("Altitude");
		alt_checkbox->setChecked(true);

		pos_covar_checkbox = new QCheckBox("Position Covariance");
	}
	else if(sensor_type == "IMU")
	{
		topic_name = Globals::DEFAULT_IMU_TOPIC.c_str();

		roll_checkbox = new QCheckBox("Roll");
		roll_checkbox->setChecked(true);

		pitch_checkbox = new QCheckBox("Pitch");
		pitch_checkbox->setChecked(true);

		yaw_checkbox = new QCheckBox("Yaw");
		yaw_checkbox->setChecked(true);

		ang_vel_checkbox = new QCheckBox("Angular Velocity");
		ang_vel_covar_checkbox = new QCheckBox("Angular Velocity Covariance");

		lin_accel_checkbox = new QCheckBox("Linear Acceleration");
		lin_accel_covar_checkbox = new QCheckBox("Linear Acceleration Covariance");
	}
	else if(sensor_type == "Laser Rangefinder")
		topic_name = Globals::DEFAULT_LASER_TOPIC.c_str();
	else if(sensor_type == "Sonar")
		topic_name = "unknown";
	else
        topic_name = Globals::DEFAULT_DIAGNOSTIC_TOPIC.c_str();

    topic_name_lineedit->setText(topic_name);


	if(sensor_type == "Voltage")
	{
		voltage_label = new QLabel("Operating Voltage");
		voltage_lineedit = new QLineEdit;
	}

	if(sensor_type == "Temperature" || sensor_type == "Velocity"
		|| sensor_type == "Voltage")
	{
		min_label = new QLabel("Minimum");
		min_lineedit = new QLineEdit;

		max_label = new QLabel("Maximum");
		max_lineedit = new QLineEdit;
	}

	if(sensor_type == "Temperature")
	{
		units_label = new QLabel(tr("Units"));

		QStringList temp_units;
		temp_units << "Celcius" << "Fahrenheit" << "Kelvin";

		units_combobox = new QComboBox;
		units_combobox->addItems(temp_units);
	}


    cancel_add_buttonbox = new QDialogButtonBox(QDialogButtonBox::Cancel |
		QDialogButtonBox::Ok);
}

/******************************************************************************
 * Function:    createLayout
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     void
 * Description: Creates the dialog grid layout based off the sensor type.
 *****************************************************************************/
void AddSensorDialog::createLayout()
{
	int count;

	dialog_layout = new QGridLayout;
	dialog_layout->addWidget(name_label, 0, 0);
	dialog_layout->addWidget(name_lineedit, 0, 1);
	dialog_layout->addWidget(topic_name_label, 1, 0);
	dialog_layout->addWidget(topic_name_lineedit, 1, 1);

	count = 2;

	if(sensor_type == "GPS" || sensor_type == "IMU")
	{
		dialog_layout->addWidget(attributes_label, count, 0);

		count++;
	}

	if(sensor_type == "GPS")
	{
		dialog_layout->addWidget(lat_checkbox, 2, 1);
		dialog_layout->addWidget(long_checkbox, 3, 1);
		dialog_layout->addWidget(alt_checkbox, 4, 1);
		dialog_layout->addWidget(pos_covar_checkbox, 5, 1);

		count += 4;
	}
	else if(sensor_type == "IMU")
	{
		dialog_layout->addWidget(roll_checkbox, 2, 1);
        dialog_layout->addWidget(pitch_checkbox, 3, 1);
        dialog_layout->addWidget(yaw_checkbox, 4, 1);
        dialog_layout->addWidget(ang_vel_checkbox, 5, 1);
        dialog_layout->addWidget(ang_vel_covar_checkbox, 6, 1);
        dialog_layout->addWidget(lin_accel_checkbox, 7, 1);
        dialog_layout->addWidget(lin_accel_covar_checkbox, 8, 1);

		count += 7;
	}


	if(sensor_type == "Voltage")
	{
		dialog_layout->addWidget(voltage_label, count, 0);
		dialog_layout->addWidget(voltage_lineedit, count, 1);
		count++;
	}

	if(sensor_type == "Temperature" || sensor_type == "Velocity"
		|| sensor_type == "Voltage")
	{
		dialog_layout->addWidget(min_label, count, 0);
		dialog_layout->addWidget(min_lineedit, count, 1);
		count++;
		dialog_layout->addWidget(max_label, count, 0);
		dialog_layout->addWidget(max_lineedit, count, 1);
		count++;
	}

	if(sensor_type == "Temperature")
	{
		dialog_layout->addWidget(units_label, count, 0);
		dialog_layout->addWidget(units_combobox, count, 1, Qt::AlignRight);
		count++;
	}


	dialog_layout->addWidget(cancel_add_buttonbox, count, 1, Qt::AlignRight);
}

/******************************************************************************
 * Function:    getName
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     QString - the name line edit's text
 * Description: Returns the sensor's name the user entered.
 *****************************************************************************/
QString AddSensorDialog::getName() const
{
	return name_lineedit->text();
}

/******************************************************************************
 * Function:    getTopicName
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     QString - the topic name line edit's text
 * Description: Returns the sensor's topic name the user entered.
 *****************************************************************************/
QString AddSensorDialog::getTopicName() const
{
	return topic_name_lineedit->text();
}

/******************************************************************************
 * Function:    setName
 * Author:      Matt Richard
 * Parameters:  const Qstring &name - the sensor's name
 * Returns:     void
 * Description: Sets the name lineedit's text to name.
 *****************************************************************************/
void AddSensorDialog::setName(const QString &name)
{
	name_lineedit->setText(name);
}

/******************************************************************************
 * Function:    setTopicName
 * Author:      Matt Richard
 * Parameters:  const QString &topic_name - the sensor's topic name
 * Returns:     void
 * Description: Sets the topic name line edit's text to topic_name.
 *****************************************************************************/
void AddSensorDialog::setTopicName(const QString &topic_name)
{
	topic_name_lineedit->setText(topic_name);
}


/******************************************************************************
 *
 *                        GPS Specific Functions
 *
 *****************************************************************************/

/******************************************************************************
 * Function:    isLatChecked
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     bool - true if checked, otherwise false
 * Description: Returns the state of the latitude checkbox.
 *****************************************************************************/
bool AddSensorDialog::isLatChecked() const
{
	if(sensor_type == "GPS")
		return lat_checkbox->isChecked();
	return false;
}

/******************************************************************************
 * Function:    isLongChecked
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     bool - true if checked, otherwise false
 * Description: Returns the state of the longitude checkbox.
 *****************************************************************************/
bool AddSensorDialog::isLongChecked() const
{
	if(sensor_type == "GPS")
		return long_checkbox->isChecked();
	return false;
}

/******************************************************************************
 * Function:    isAltChecked
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     bool - true if checked, otherwise false
 * Description: Returns the state of the altitude checkbox.
 *****************************************************************************/
bool AddSensorDialog::isAltChecked() const
{
	if(sensor_type == "GPS")
		return alt_checkbox->isChecked();
	return false;
}

/******************************************************************************
 * Function:    isPosCovarChecked
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     bool - true if checked, otherwise false
 * Description: Returns the state of the position covariance checkbox.
 *****************************************************************************/
bool AddSensorDialog::isPosCovarChecked() const
{
	if(sensor_type == "GPS")
		return pos_covar_checkbox->isChecked();
	return false;
}


/******************************************************************************
 *
 *                       IMU Specific Funtions
 *
 *****************************************************************************/

/******************************************************************************
 * Function:    isRollChecked
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     bool - true if checked, otherwise false
 * Description: Returns the state of the roll checkbox.
 *****************************************************************************/
bool AddSensorDialog::isRollChecked() const
{
	if(sensor_type == "IMU")
		return roll_checkbox->isChecked();
	return false;
}

/******************************************************************************
 * Function:    isPitchChecked
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     bool - true if checked, otherwise false
 * Description: Returns the state of the pitch checkbox.
 *****************************************************************************/
bool AddSensorDialog::isPitchChecked() const
{
    if(sensor_type == "IMU")
        return pitch_checkbox->isChecked();
    return false;
}

/******************************************************************************
 * Function:    isYawChecked
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     bool - true if checked, otherwise false
 * Description: Returns the state of the yaw checkbox.
 *****************************************************************************/
bool AddSensorDialog::isYawChecked() const
{
    if(sensor_type == "IMU")
        return yaw_checkbox->isChecked();
    return false;
}

/******************************************************************************
 * Function:    isAngVelChecked
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     bool - true if checked, otherwise false
 * Description: Returns the state of the angular velocity checkbox.
 *****************************************************************************/
bool AddSensorDialog::isAngVelChecked() const
{
    if(sensor_type == "IMU")
        return ang_vel_checkbox->isChecked();
    return false;
}

/******************************************************************************
 * Function:    isAngVelCovarChecked
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     bool - true if checked, otherwise false
 * Description: Returns the state of the angular velocity covariance checkbox.
 *****************************************************************************/
bool AddSensorDialog::isAngVelCovarChecked() const
{
    if(sensor_type == "IMU")
        return ang_vel_covar_checkbox->isChecked();
    return false;
}

/******************************************************************************
 * Function:    isLinAccelChecked
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     bool - true if checked, otherwise false
 * Description: Returns the state of the linear acceleration checkbox.
 *****************************************************************************/
bool AddSensorDialog::isLinAccelChecked() const
{
    if(sensor_type == "IMU")
        return lin_accel_checkbox->isChecked();
    return false;
}

/******************************************************************************
 * Function:    isLinAccelCovarChecked
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     bool - true if checked, otherwise false
 * Description: Returns the state of the linear acceleration covariance
 *              checkbox.
 *****************************************************************************/
bool AddSensorDialog::isLinAccelCovarChecked() const
{
    if(sensor_type == "IMU")
        return lin_accel_covar_checkbox->isChecked();
    return false;
}


/******************************************************************************
 *
 *      Temperature, Velocity, and Voltage Specific Functions
 *
 *****************************************************************************/

/******************************************************************************
 * Function:    getMin
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     QString - value the user entered
 * Description: Returns the text contained in the minimun line edit.
 *****************************************************************************/
QString AddSensorDialog::getMin() const
{
	if(sensor_type == "Temperature" || sensor_type == "Velocity" ||
	   sensor_type == "Voltage")
		return min_lineedit->text();
	return "";
}

/******************************************************************************
 * Function:    getMax
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     QString - value the user entered
 * Description: Returns the text contained in the maximum line edit.
 *****************************************************************************/
QString AddSensorDialog::getMax() const
{
    if(sensor_type == "Temperature" || sensor_type == "Velocity" ||
       sensor_type == "Voltage")
        return max_lineedit->text();
    return "";
}


/******************************************************************************
 *
 *                  Temperature Specific Functions
 *
 *****************************************************************************/

/******************************************************************************
 * Function:    getUnits
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     QString - current units combobox text
 * Description: Returns the units the user selects from the units combobox.
 *****************************************************************************/
QString AddSensorDialog::getUnits() const
{
	if(sensor_type == "Temperature")
		return units_combobox->currentText();
	return "";
}


/******************************************************************************
 *
 *                      Voltage Specific Functions
 *
 *****************************************************************************/

/******************************************************************************
 * Function:    getVoltage
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     QString - the voltage entered in by the user
 * Description: Returns the voltage value the user types into the voltage
 *              line edit.
 *****************************************************************************/
QString AddSensorDialog::getVoltage() const
{
	if(sensor_type == "Voltage")
		return voltage_lineedit->text();
	return "";
}


////////////////////////////// Joint Dialog /////////////////////////////////

JointDialog::JointDialog(QWidget *parent) : QDialog(parent)
{
    createDialog();
}

JointDialog::JointDialog(const QString &joint_name, const QString &display_name,
    QWidget *parent) : QDialog(parent)
{
    createDialog();

    joint_name_lineedit->setText(joint_name);
    display_name_lineedit->setText(display_name);
}

void JointDialog::createDialog()
{
    QLabel *joint_name_label = new QLabel(tr("Joint Name"));
    joint_name_lineedit = new QLineEdit;

    QLabel *display_name_label = new QLabel(tr("Display Name"));
    display_name_lineedit = new QLineEdit;

    button_box = new QDialogButtonBox(QDialogButtonBox::Cancel | 
                                      QDialogButtonBox::Ok);

    connect(button_box, SIGNAL(accepted()), this, SLOT(accept()));
    connect(button_box, SIGNAL(rejected()), this, SLOT(reject()));

    QGridLayout *dialog_layout = new QGridLayout;
    dialog_layout->addWidget(joint_name_label, 0, 0);
    dialog_layout->addWidget(joint_name_lineedit, 0, 1);
    dialog_layout->addWidget(display_name_label, 1, 0);
    dialog_layout->addWidget(display_name_lineedit, 1, 1);
    dialog_layout->addWidget(button_box, 2, 1);
    dialog_layout->setSizeConstraint(QLayout::SetFixedSize);
    setLayout(dialog_layout);
}
