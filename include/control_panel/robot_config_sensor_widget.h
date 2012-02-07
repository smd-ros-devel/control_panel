/******************************************************************************
 * robot_config_sensor_widget.h
 *
 * Author:      Matt Richard
 * Date:        Jan 9, 2012
 * Description:
 *****************************************************************************/

#ifndef CONTROL_PANEL_ROBOT_CONFIG_SENSOR_WIDGET_H
#define CONTROL_PANEL_ROBOT_CONFIG_SENSOR_WIDGET_H

#include <QFrame>

QT_BEGIN_NAMESPACE
class QCheckBox;
class QLabel;
class QHBoxLayout;
QT_END_NAMESPACE

#include <QString>
#include <string>

class RobotConfigSensorWidget : public QFrame
{
	Q_OBJECT

	public:
		RobotConfigSensorWidget(QWidget *parent = 0);
        QString getSensorType() const;
        QString getSensorName() const;
		bool isSelected() const;
		void setSensorType(const QString &type);
		void setSensorName(const QString &name);

	private:
		QHBoxLayout *sensor_widget_layout;

		QCheckBox *selected_checkbox;

		QLabel *sensor_type_label;
		QLabel *sensor_name_label;

		QString sensor_type;
		QString sensor_name;
};

#endif // CONTROL_PANEL_ROBOT_CONFIG_SENSOR_WIDGET_H
