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
 * \file   image_viewer.h
 * \date   Jan 4, 2012
 * \author Matt Richard
 */
#ifndef CONTROL_PANEL_IMAGE_VIEWER_H
#define CONTROL_PANEL_IMAGE_VIEWER_H

#include <QGraphicsView>
#include <QPixmap>

QT_BEGIN_NAMESPACE
class QGraphicsEllipseItem;
class QGraphicsPixmapItem;
class QGraphicsScene;
class QGraphicsTextItem;
class QPainter;
class QPen;
class QRectF;
class QWheelEvent;
QT_END_NAMESPACE

/**
 * \class ImageViewer
 * \brief A graphics view widget for displaying a robots video, LiDAR, and maps.
 *
 * The image viewer allows for a grip to be overlayed on top of the image,
 * zooming, and translating the image.
 */
class ImageViewer : public QGraphicsView
{
	Q_OBJECT

	public:
		/**
		 * \brief
		 *
		 * \param parent The parent widget.
		 */
		ImageViewer(QWidget *parent = 0);
		QPixmap getImagePixmap() const { return image_pixmap; }
		bool gridVisible() const { return grid_visible; }
		void setGridLineInterval(int pixels);
		void setImagePixmap(const QPixmap &pixmap, int interval = -1);
		void setImageVisible(bool visible);
		void setRobotPosition(double x_pos, double y_pos);

	public slots:
		void setScale(int factor);
		void showGrid(bool show);
		void showOdometry(bool show);

	signals:
		void scaleChanged(int factor);

	protected:
		void drawForeground(QPainter *painter, const QRectF &rect);
		void wheelEvent(QWheelEvent *event);

	private:
		QGraphicsScene *scene;
		QGraphicsPixmapItem *image_item;
		QGraphicsTextItem *text_item;
		QGraphicsEllipseItem *robot_pos_item;

		QPixmap image_pixmap;

		bool show_odom;
		bool grid_visible;
		int grid_interval;
		QPen grid_pen;
		float scale_factor;
};

#endif // CONTROL_PANEL_IMAGE_VIEWER_H
