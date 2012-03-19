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
    //setInteractive(false);

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
