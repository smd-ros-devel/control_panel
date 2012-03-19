/*
 * Copyright (c) 2011, 2012 SDSM&T RIAS.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


/**
 * \file   artificial_horizon.cpp
 * \date   July 17, 2011
 * \author Matt Richard
 */
#include <QtGui>
#include "control_panel/widgets/artificial_horizon.h"

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
