/******************************************************************************
 * heading_indicator.cpp
 *
 * Author:      Matt Richard
 * Date:        July 14, 2011
 * Description: 
 *****************************************************************************/

#include <QtGui>

#include "control_panel/widgets/heading_indicator.h"


/******************************************************************************
 * Function:    HeadingIndicator
 * Author:      Matt Richard
 * Parameters:  QWidget *parent - 
 * Returns:     None
 * Description: Constructor.
 *****************************************************************************/
HeadingIndicator::HeadingIndicator(QWidget *parent)
    : QGraphicsView(parent)
{
    // Background
    background_item = new QGraphicsPixmapItem(QPixmap(
        ":/images/heading_indicator/compass_background.png").scaled(
        100, 100, Qt::KeepAspectRatio, Qt::SmoothTransformation));
    background_item->setTransformOriginPoint(
        background_item->pixmap().width() / 2.0,
        background_item->pixmap().height() / 2.0);

    // Cover
    cover_item = new QGraphicsPixmapItem(QPixmap(
        ":/images/heading_indicator/compass_cover.png").scaled(
        100, 100, Qt::KeepAspectRatio, Qt::SmoothTransformation));

    indicator_item = new QGraphicsPixmapItem(QPixmap(
        ":/images/heading_indicator/compass_indicator.png").scaled(
        100, 100, Qt::KeepAspectRatio, Qt::SmoothTransformation));


    // Create the scene
    scene = new QGraphicsScene(this);
    scene->addItem(background_item);
    scene->addItem(indicator_item);
    scene->addItem(cover_item);


    // Set graphics view's properties
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);
    setFrameStyle(QFrame::NoFrame | QFrame::Plain);
    setBackgroundRole(QPalette::Window);
    setAutoFillBackground(true);
    setScene(scene);

    // Initialize heading indicator
    setYaw(0.0);
}

/******************************************************************************
 * Function:    getYaw
 * Author:      Matt Richard
 * Parameters:  None
 * Returns:     double - 
 * Description: 
 *****************************************************************************/
double HeadingIndicator::getYaw() const
{
    return yaw;
}

/******************************************************************************
 * Function:    setYaw
 * Author:      Matt Richard
 * Parameters:  double angle - 
 * Returns:     void
 * Description: 
 *****************************************************************************/
void HeadingIndicator::setYaw(double angle)
{
    yaw = angle;

    // Update the heading indicator
    background_item->setRotation(yaw);
}
