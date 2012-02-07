/******************************************************************************
 * robot_config_sensor_widget.cpp
 *
 * Author:      Matt Richard
 * Date:        Jan 9, 2012
 * Description:
 *****************************************************************************/

#include <QtGui>
#include "control_panel/robot_config_sensor_widget.h"

/******************************************************************************
 * Function:    RobotConfigSensorWidget
 * Author:      Matt Richard
 * Parameters:  QWidget *parent - 
 * Returns:     None
 * Description: Constructor.
 *****************************************************************************/
RobotConfigSensorWidget::RobotConfigSensorWidget(QWidget *parent)
	: QFrame(parent)
{
	sensor_type_label = new QLabel;
	sensor_name_label = new QLabel;

	selected_checkbox = new QCheckBox;

	sensor_widget_layout = new QHBoxLayout;
	sensor_widget_layout->addWidget(selected_checkbox, 0, Qt::AlignLeft);
	sensor_widget_layout->addWidget(sensor_type_label, 0, Qt::AlignLeft);
	sensor_widget_layout->addWidget(sensor_name_label, 0, Qt::AlignLeft);
	sensor_widget_layout->addStretch();

	setLayout(sensor_widget_layout);
	setFixedHeight(40);
	setFrameStyle(QFrame::Box | QFrame::Sunken);
}

/******************************************************************************
 * Function:    getSensorType
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     QString - 
 * Description:
 *****************************************************************************/
QString RobotConfigSensorWidget::getSensorType() const
{
	return sensor_type;
}

/******************************************************************************
 * Function:    getSensorName
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     QString - 
 * Description: 
 *****************************************************************************/
QString RobotConfigSensorWidget::getSensorName() const
{
	return sensor_name;
}

/******************************************************************************
 * Function:    isSelected
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     bool - 
 * Description:
 *****************************************************************************/
bool RobotConfigSensorWidget::isSelected() const
{
	return selected_checkbox->isChecked();
}

/******************************************************************************
 * Function:    setSensorType
 * Author:      Matt Richard
 * Parameters:  const char *type - 
 * Returns:     void
 * Description:
 *****************************************************************************/
void RobotConfigSensorWidget::setSensorType(const QString &type)
{
	sensor_type = type;

	sensor_type_label->setText(sensor_type + tr(": "));
}

/******************************************************************************
 * Function:    setSensorName
 * Author:      Matt Richard
 * Parameters:  const char *name - 
 * Returns:     void
 * Description:
 *****************************************************************************/
void RobotConfigSensorWidget::setSensorName(const QString &name)
{
	sensor_name = name;

	sensor_name_label->setText(sensor_name);
}
