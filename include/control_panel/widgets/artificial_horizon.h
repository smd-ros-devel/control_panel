/******************************************************************************
** artificial_horizon.h
**
** Author:      Matt Richard
** Date:        July 17, 2011
** Description:
******************************************************************************/

#ifndef CONTROL_PANEL_ARTIFICIAL_HORIZON_H
#define CONTROL_PANEL_ARTIFICIAL_HORIZON_H

#include <QGraphicsPixmapItem>


class ArtificialHorizon : public QGraphicsPixmapItem
{
    public:
        ArtificialHorizon();
        QRectF boundingRect() const;
        QPainterPath shape() const;
        void updateArtificialHorizon(int roll_angle, int pitch_angle);

    private:
        float center_x;
        float center_y;
        int offset;
        int scene_width;
        int scene_height;

        int roll;
        int pitch;
};

#endif // CONTROL_PANEL_ARTIFICIAL_HORIZON_H
