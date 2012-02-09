/*****************************************************************************
** data_pane.h
**
** Author:      Matt Richard
** Date:        June 2011
** Description:
*****************************************************************************/

#ifndef CONTROL_PANEL_DATA_PANE_H
#define CONTROL_PANEL_DATA_PANE_H

#include <QWidget>
#include <QList>

QT_BEGIN_NAMESPACE
class QImage;
class QLabel;
//class QList<QLabel *>;
//class QQuaternion;
class QPushButton;
class QHBoxLayout;
QT_END_NAMESPACE

#include <stdio.h>
#include <cmath>

//#include "robot_config.h"
//#include "widgets/attitude_indicator.h"
#include "widgets/battery_display.h"
//#include "widgets/heading_indicator.h"
#include "widgets/imu_display.h"
#include "widgets/gps_display.h"
#include "widgets/odometry_display.h"
#include "widgets/joint_state_display.h"
#include "globals.h"


class DataPane : public QWidget
{
    Q_OBJECT

    public:
        DataPane(QWidget *parent = 0);
        GpsDisplay *addGpsDisplay(const QString &name, bool lat = true,
            bool lon = true, bool alt = false);
        ImuDisplay *addImuDisplay(const QString &name, bool roll = true,
            bool pitch = true, bool yaw = true, bool ang_vel = false,
            bool lin_accel = false, bool heading_graphic = true,
            bool attitude_graphic = true);
        JointStateDisplay *addJointStateDisplay(const QString &widget_name,
            const QStringList &names, const QStringList &display_names,
            bool show_pos = true, bool show_vel = true, bool show_eff = false);
        OdometryDisplay *addOdometryDisplay(const QString &name, bool pos = true,
            bool rpy = true, bool lin_vel = true, bool ang_vel = true,
            bool heading_graphic = true, bool attitude_graphic = true);
        void showRangeLabel(bool show);
        void setRCModeText(const QString &mode);

    public slots:
        void takeoffLandButtonClicked();
        void connectionStatusChanged(int new_status);
        void updateBatteryData(float battery_data);
        void updateRange(float range);
        //void updateImuData(const QQuaternion &imu_data);
        //void updateGpsData(double latitude, double longitude, double altitude);
        //void updateWheelData(double wheelFR, double wheelFL, double wheelRR, double wheelRL);

    signals:
        void takeoff();
        void land();

    private:
        void createLabels();
        void initializeLabels();
        void createLayout();

        QHBoxLayout *data_pane_layout;

//        AttitudeIndicator *attitude_indicator;
        BatteryDisplay *battery_display;
//        HeadingIndicator *heading_indicator;

        QImage *status_light;

        QPushButton *takeoff_land_button;

        GpsDisplay *gps_display;
        ImuDisplay *imu_display;
        JointStateDisplay *joint_state_display;
        OdometryDisplay *odom_display;
        QList<ImuDisplay *> *imu_list;
        QList<GpsDisplay *> *gps_list;
        QList<JointStateDisplay *> *joint_state_list;
        QList<OdometryDisplay *> *odom_list;
        //QList<QLabel *> *label_list;
        //QList<QLabel *> *gps_list;
        //QList<QLabel *> *imu_list;
        //QList<QLabel *> *joint_list;
        //QList<QLabel *> *range_list;
        //QList<QLabel *> *diagnostic_list;
        //QList<BatteryDisplay *> *battery_display_list;

        QLabel *status_light_label;
        QLabel *connection_status_label;
        QLabel *rc_mode_label;

        bool use_range;
        QLabel *range_label;
/*
        QLabel *battery_status_label;
        QLabel *heading_label;
        QLabel *attitude_label;
        QLabel *roll_label;
        QLabel *pitch_label;
        QLabel *yaw_label;
        QLabel *altitude_label;
        QLabel *wheelFR_label;
        QLabel *wheelFL_label;
        QLabel *wheelRR_label;
        QLabel *wheelRL_label;
        QLabel *speed_label;
        QLabel *turn_label;
        QLabel *slip_label;
        QLabel *slide_label;
        QLabel *gps_label;
        QLabel *radiation_label;
        QLabel *temp_label;
        QLabel *light_label;
        QLabel *pressure_label;
*/
};

#endif // CONTROL_PANEL_DATA_PANE_H
