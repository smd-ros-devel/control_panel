/**
 * odometry_display.h
 *
 * Author:      Matt Richard
 * Date:        Jan 26, 2012
 * Description: Displays odometry information (position, orientation,
 *              linear velocity, and angular velocity).
 **/

#ifndef CONTROL_PANEL_ODOMETRY_DISPLAY_H
#define CONTROL_PANEL_ODOMETRY_DISPLAY_H

#include <QWidget>
#include <QQuaternion>
#include <QVector3D>
#include <QString>

QT_BEGIN_NAMESPACE
class QHBoxLayout;
class QLabel;
QT_END_NAMESPACE

#include "attitude_indicator.h"
#include "heading_indicator.h"
#include "control_panel/globals.h"

class OdometryDisplay : public QWidget
{
    Q_OBJECT

    public:
        OdometryDisplay(QWidget *parent = 0);
        OdometryDisplay(const QString &name, bool show_pos = true,
            bool show_rpy = true, bool show_lin_vel = true,
            bool show_ang_vel = true, bool show_heading = true,
            bool show_attitude = true, QWidget *parent = 0);
        void zeroValues();

    public slots:
        void updateOdometryDisplay(const QVector3D &position,
            const QQuaternion &orientation, const QVector3D &linear_velocity,
            const QVector3D &angular_velocity);

    private:
        void createWidget();

        QHBoxLayout *widget_layout;

        QString odometry_name;

        bool use_pos;
        bool use_rpy;
        bool use_lin_vel;
        bool use_ang_vel;
        bool use_heading_ind;
        bool use_attitude_ind;

        double roll;
        double pitch;
        double yaw;
        QVector3D pos;
        QVector3D lin_vel;
        QVector3D ang_vel;

        AttitudeIndicator *attitude;
        HeadingIndicator *heading;

        QLabel *name_label;
        QLabel *position_label;
        QLabel *roll_label;
        QLabel *pitch_label;
        QLabel *yaw_label;
        QLabel *lin_vel_label;
        QLabel *ang_vel_label;
};

#endif // CONTROL_PANEL_ODOMETRY_DISIPLAY_H
