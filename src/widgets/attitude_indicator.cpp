/* @todo Add license here */


/**
 * \file   attitude_indicator.cpp
 * \date   July 15, 2011
 * \author Matt Richard
 */
#include <QtGui>
#include "control_panel/widgets/attitude_indicator.h"


AttitudeIndicator::AttitudeIndicator(QWidget *parent)
    : QGraphicsView(parent)
{
    // Set graphics view properties
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);
    setFrameStyle(QFrame::NoFrame | QFrame::Plain);
    setBackgroundRole(QPalette::Window);
    setAutoFillBackground(true);
    setInteractive(false);


    // Create graphics items
    QPixmap crosshair_pixmap = 
        QPixmap(":/images/attitude_indicator/crosshair.png");

    crosshair_item = new QGraphicsPixmapItem;
    crosshair_item->setPixmap(crosshair_pixmap);

    artificial_horizon_item = new ArtificialHorizon();


    cover_item = new QGraphicsPixmapItem(
        QPixmap(":/images/status_lights/status_light_cover.png"));


    // Create graphics scene
    scene = new QGraphicsScene(this);
//    scene->setSceneRect(0,
//                        0,
//                        crosshair_pixmap.width(),
//                        crosshair_pixmap.height());
    scene->addItem(artificial_horizon_item);
    scene->addItem(crosshair_item);
    scene->addItem(cover_item);
    scene->setSceneRect(scene->itemsBoundingRect());

    setScene(scene);
//    ensureVisible(crosshair_item);
    setFixedSize(scene->sceneRect().toRect().size());


    // Initialize attitute
    setAttitude(0.0, 0.0);
}

/******************************************************************************
** Function:    getRoll
** Author:      Matt Richard
** Parameters:  None
** Returns:     double
** Description:
******************************************************************************/
double AttitudeIndicator::getRoll() const
{
    return attitude_data.roll;
}

/******************************************************************************
** Function:    getPitch
** Author:      Matt Richard
** Parameters:  None
** Returns:     double
** Description: 
******************************************************************************/
double AttitudeIndicator::getPitch() const
{
    return attitude_data.pitch;
}

/******************************************************************************
** Function:    setRoll
** Author:      Matt Richard
** Parameters:  double angle -
** Returns:     void
** Description: 
******************************************************************************/
void AttitudeIndicator::setRoll(double angle)
{
    attitude_data.roll = angle;

    updateAttitudeIndicator();
}

/******************************************************************************
** Function:    setPitch
** Author:      Matt Richard
** Parameters:  double angle -
** Returns:     void
** Description: 
******************************************************************************/
void AttitudeIndicator::setPitch(double angle)
{
    attitude_data.pitch = angle;

    updateAttitudeIndicator();
}

/******************************************************************************
** Function:    setAttitude
** Author:      Matt Richard
** Parameters:  double roll_angle -
**              double pitch_angle -
** Returns:     void
** Description: 
******************************************************************************/
void AttitudeIndicator::setAttitude(double roll_angle, double pitch_angle)
{
    attitude_data.roll = roll_angle;
    attitude_data.pitch = pitch_angle;

    updateAttitudeIndicator();
}

/******************************************************************************
** Function:    updateAttitudeIndicator
** Author:      Matt Richard
** Parameters:  None
** Returns:     void
** Description: 
******************************************************************************/
void AttitudeIndicator::updateAttitudeIndicator()
{
    artificial_horizon_item->updateArtificialHorizon(
        attitude_data.roll, attitude_data.pitch);
}
