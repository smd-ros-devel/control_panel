/**
 * gps_display.cpp
 *
 * Author:      Matt Richard
 * Date:        Jan 20, 2012
 * Description:
 **/

#include <QtGui>
#include "control_panel/widgets/gps_display.h"

/**
 * This constructor assumes use of all labels
 **/
GpsDisplay::GpsDisplay(QWidget *parent)
{
    gps_name = "GPS";

    use_lat = true;
    use_long = true;
    use_alt = true;

    createWidget();
}

/**
 * Allows only certain labels to be displayed
 **/
GpsDisplay::GpsDisplay(const QString &name, bool show_lat, bool show_long,
    bool show_alt, QWidget *parent) : QWidget(parent)
{
    if(name == "")
        gps_name = "GPS";
    else
        gps_name = name;

    use_lat = show_lat;
    use_long = show_long;
    use_alt = show_alt;

    createWidget();
}

/**
 * Initializes widgets and creates layout
 **/
void GpsDisplay::createWidget()
{
    // counter for the grid layout
    int rows = 0;

    // initialize priavte variables
    zeroValues();

    widget_layout = new QHBoxLayout;
    //widget_layout->addStretch();

    // Grid layout for labels
    QGridLayout *gps_gridlayout = new QGridLayout;

    if(use_lat || use_long || use_alt)
    {
        name_label = new QLabel(gps_name);

        gps_gridlayout->addWidget(name_label, rows, 0, 1, 0);
        rows++;

        gps_gridlayout->setColumnMinimumWidth(1, 8);
    }

    // Create latitude label and add to layout
    if(use_lat)
    {
        QLabel *lat_str = new QLabel("Latitude:");
        latitude_label = new QLabel;

        gps_gridlayout->addWidget(lat_str, rows, 0, Qt::AlignLeft);
        gps_gridlayout->addWidget(latitude_label, rows, 1, Qt::AlignLeft);
        rows++;
    }

    // Create longitude label and add to layout
    if(use_long)
    {
        QLabel *long_str = new QLabel("Longitude:");
        longitude_label = new QLabel;

        gps_gridlayout->addWidget(long_str, rows, 0, Qt::AlignLeft);
        gps_gridlayout->addWidget(longitude_label, rows, 1, Qt::AlignLeft);
        rows++;
    }

    // Create altitude label and add to layout
    if(use_alt)
    {
        QLabel *alt_str = new QLabel("Altitude:");
        altitude_label = new QLabel;

        gps_gridlayout->addWidget(alt_str, rows, 0, Qt::AlignLeft);
        gps_gridlayout->addWidget(altitude_label, rows, 1, Qt::AlignLeft);
        rows++;
    }

    widget_layout->addLayout(gps_gridlayout);
    //widget_layout->addStretch();

    setLayout(widget_layout);
}

/**
 * Zeros all private data variables
 **/
void GpsDisplay::zeroValues()
{
    latitude = 0.0;
    longitude = 0.0;
    altitude = 0.0;
}

void GpsDisplay::updateGpsDisplay(double lat, double lon, double alt)
{
    char direction;

    latitude = lat;
    longitude = lon;
    altitude = alt;

    // Update latitude
    if(use_lat)
    {
        // Check latitude direction
        if(lat >= 0)
            direction = 'N';
        else
        {
            direction = 'S';
            lat *= -1;
        }

        latitude_label->setText(QString("%1 ").arg(lat, 0, 'f', 3) +
            Globals::DegreesSymbol + direction);
    }

    // Update longitude
    if(use_long)
    {
        // Check longitude direction
        if(lon >= 0)
            direction = 'E';
        else
        {
            direction = 'W';
            lon *= -1;
        }

        longitude_label->setText(QString("%1 ").arg(lon, 0, 'f', 3) +
            Globals::DegreesSymbol + direction);
    }

    // Update altitude
    if(use_alt)
        altitude_label->setText(QString("%1 ").arg(alt, 0, 'f', 3) + 'm');
}
