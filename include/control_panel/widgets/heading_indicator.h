/******************************************************************************
 * heading_indicator.h
 *
 * Author:      Matt Richard
 * Date:        July 14, 2011
 * Description: A graphic to display the yaw of a robot.
 *****************************************************************************/

#ifndef CONTROL_PANEL_HEADING_INDICATOR_H
#define CONTROL_PANEL_HEADING_INDICATOR_H

#include <QGraphicsView>

QT_BEGIN_NAMESPACE
class QGraphicsScene;
class QGraphicsPixmapItem;
QT_END_NAMESPACE


/**
 * \class  HeadingIndicator
 * \author Matt Richard
 * \brief  A graphic to display the yaw of a robot.
 */
class HeadingIndicator : public QGraphicsView
{
    public:
        HeadingIndicator(QWidget *parent = 0);
        double getYaw() const;
        void setYaw(double angle);


    private:
        QGraphicsScene *scene;
        QGraphicsPixmapItem *background_item;
        QGraphicsPixmapItem *indicator_item;
        QGraphicsPixmapItem *cover_item;
        double yaw;
};

#endif // CONTROL_PANEL_HEADING_INDICATOR_H
