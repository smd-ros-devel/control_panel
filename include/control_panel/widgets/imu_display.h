/**
 * imu_display.h
 *
 * Author:      Matt Richard
 * Date:        Jan 18, 2012
 * Description: 
 **/

#ifndef CONTROL_PANEL_IMU_DISPLAY_H
#define CONTROL_PANEL_IMU_DISPLAY_H

#include <QWidget>
#include <QVector3D>
#include <QString>

QT_BEGIN_NAMESPACE
class QLabel;
class QQuaternion;
class QHBoxLayout;
QT_END_NAMESPACE

#include "attitude_indicator.h"
#include "heading_indicator.h"
#include "control_panel/globals.h"

class ImuDisplay : public QWidget
{
    Q_OBJECT

    public:
        ImuDisplay(QWidget *parent = 0);
        ImuDisplay(const QString &name, bool show_roll = true,
            bool show_pitch = true, bool show_yaw = true,
            bool show_ang_vel = false, bool show_lin_accel = false,
            bool show_attitude = true, bool show_heading = true,
            QWidget *parent = 0);
        void zeroValues();
        //void showAttitudeIndicator(bool show);
        //void showHeadingIndicator(bool show);
        //void showAngularVelocity(bool show);
        //void showLinearAcceleration(bool show);

    public slots:
        void updateImuDisplay(const QQuaternion &orientation);
        void updateImuDisplay(const QQuaternion &orientation,
                              const QVector3D &ang_vel,
                              const QVector3D &lin_accel);
        //void updateImuDisplay(const QQuaternion &orientation, double ori_covar[9],
        //                      const QVector3D &ang_vel, double ang_vel_covar[9],
        //                      const QVector3D &lin_accel, double lin_accel_covar[9]);


    private:
        void createWidget();

        QHBoxLayout *widget_layout;

        QString imu_name;

        bool use_roll;
        bool use_pitch;
        bool use_yaw;
        bool use_ang_vel;
        bool use_lin_accel;
        bool use_attitude_ind;
        bool use_heading_ind;

        double roll;
        double pitch;
        double yaw;
        QVector3D angular_velocity;
        QVector3D linear_acceleration;

        AttitudeIndicator *attitude;
        HeadingIndicator *heading;

        QLabel *name_label;
        QLabel *roll_label;
        QLabel *pitch_label;
        QLabel *yaw_label;
        QLabel *ang_vel_label;
        QLabel *lin_accel_label;
};

#endif // CONTROL_PANEL_IMU_DISPLAY_H
