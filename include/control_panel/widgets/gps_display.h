/**
 * gps_display.h
 *
 * Author:      Matt Richard
 * Date:        Jan 20, 2012
 * Description:
 **/

#ifndef CONTROL_PANEL_GPS_DISPLAY_H
#define CONTROL_PANEL_GPS_DISPLAY_H

#include <QWidget>
#include <QString>

QT_BEGIN_NAMESPACE
class QHBoxLayout;
class QLabel;
QT_END_NAMESPACE

#include "control_panel/globals.h"

class GpsDisplay : public QWidget
{
    Q_OBJECT

    public:
        GpsDisplay(QWidget *parent = 0);
        GpsDisplay(const QString &name, bool show_lat = true,
            bool show_long = true, bool show_alt = false, QWidget *parent = 0);
        void zeroValues();

    public slots:
        void updateGpsDisplay(double lat, double lon, double alt = 0.0);

    private:
        void createWidget();

        QHBoxLayout *widget_layout;

        QString gps_name;

        bool use_lat;
        bool use_long;
        bool use_alt;

        double latitude;
        double longitude;
        double altitude;

        QLabel *name_label;
        QLabel *latitude_label;
        QLabel *longitude_label;
        QLabel *altitude_label;
};

#endif // CONTROL_PANEL_GPS_DISPLAY_H
