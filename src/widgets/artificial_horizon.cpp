/******************************************************************************
** artificial_horizon.cpp
**
** Author:      Matt Richard
** Date:        July 17, 2011
** Description: 
******************************************************************************/

#include <QtGui>

#include "control_panel/widgets/artificial_horizon.h"

/******************************************************************************
** Function:    ArtificialHorizon
** Author:      Matt Richard
** Parameters:  None
** Returns:     None
** Description: Constructor
******************************************************************************/
ArtificialHorizon::ArtificialHorizon()
{
    setFlag(QGraphicsItem::ItemClipsToShape);

    QPixmap artificial_horizon_pixmap(":/images/attitude_indicator/artificial_horizon.png");

    setPixmap(artificial_horizon_pixmap);

    center_x = artificial_horizon_pixmap.width() / 2.0;
    center_y = artificial_horizon_pixmap.height() / 2.0;
    offset = 50;
    scene_width = 100;
    scene_height = 100;
    roll = 0;
    pitch = 0;


    // Initialize artificial horizon
    setTransformOriginPoint(center_x, center_y);
    setPos(-1.0 * center_x + offset, -1.0 * center_y + offset);
}

/******************************************************************************
** Function:    boundingRect
** Author:      Matt Richard
** Parameters:  None
** Returns:     QRectF - the bounding rectangle
** Description:
******************************************************************************/
QRectF ArtificialHorizon::boundingRect() const
{
    return QRectF(center_x - offset, center_y - offset - pitch,
                  scene_width, scene_height);
}

/******************************************************************************
** Function:    shape
** Author:      Matt Richard
** Parameters:  None
** Returns:     QPainterPath - 
** Description:
******************************************************************************/
QPainterPath ArtificialHorizon::shape() const
{
    QPainterPath path;
    path.addEllipse(boundingRect());

    return path;
}

/******************************************************************************
** Function:    updateArtificialHorizon
** Author:      Matt Richard
** Parameters:  int roll_angle -
**              int pitch angle - 
** Returns:     void
** Description: 
******************************************************************************/
void ArtificialHorizon::updateArtificialHorizon(int roll_angle, int pitch_angle)
{
    roll = roll_angle;
    pitch = pitch_angle;

    setPos(x(), -1.0 * center_y + offset + pitch);

    setTransformOriginPoint(center_x, center_y - pitch);

    setRotation(roll);
}
