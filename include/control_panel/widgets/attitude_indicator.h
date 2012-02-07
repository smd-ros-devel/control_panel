/******************************************************************************
** attitude_indicator.h
**
** Author:      Matt Richard
** Date:        July 15, 2011
** Description: 
******************************************************************************/

#ifndef CONTROL_PANEL_ATTITUDE_INDICATOR_H
#define CONTROL_PANEL_ATTITUDE_INDICATOR_H

#include <QGraphicsView>

QT_BEGIN_NAMESPACE
class QGraphicsScene;
class QGraphicsPixmapItem;
QT_END_NAMESPACE

#include "artificial_horizon.h"


struct Attitude
{
    double roll;  // -180 to 180 degrees
    double pitch; //  -90 to  90 degrees
    //double altitude;
};


class AttitudeIndicator : public QGraphicsView
{
    Q_OBJECT

    public:
        AttitudeIndicator(QWidget *parent = 0);
        double getRoll() const;
        double getPitch() const;
        void setRoll(double angle);
        void setPitch(double angle);
        void setAttitude(double roll_angle, double pitch_angle);

    private:
        void updateAttitudeIndicator();

        QGraphicsScene *scene;
        QGraphicsPixmapItem *crosshair_item;
        QGraphicsPixmapItem *cover_item;
        ArtificialHorizon *artificial_horizon_item;
        Attitude attitude_data;
};

#endif // CONTROL_PANEL_ATTITUDE_INDICATOR_H
