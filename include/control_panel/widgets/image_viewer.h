/******************************************************************************
 * image_viewer.h
 *
 * Author:      Matt Richard
 * Date:        Jan 4, 2012
 * Description: A graphics view widget for displaying a robots video, LiDAR,
 *              and map feeds. The view allows for a grip to be overlayed on
 *              top of the image, zooming in and out, and moving the image
 *              around.
 *****************************************************************************/

#ifndef CONTROL_PANEL_IMAGE_VIEWER_H
#define CONTROL_PANEL_IMAGE_VIEWER_H

#include <QGraphicsView>
#include <QPixmap>

QT_BEGIN_NAMESPACE
class QGraphicsPixmapItem;
class QGraphicsScene;
class QGraphicsTextItem;
class QPainter;
class QPen;
class QRectF;
class QWheelEvent;
QT_END_NAMESPACE

#include <string>
#include <stdio.h>


class ImageViewer : public QGraphicsView
{
	Q_OBJECT

	public:
		ImageViewer(QWidget *parent = 0);
		QPixmap getImagePixmap() const;
		bool gridVisible() const;
		void setGridLineInterval(int pixels);
		void setImagePixmap(const QPixmap &pixmap, int interval = -1);
		void setImageVisible(bool visible);

	public slots:
		void setScale(int factor);
        void showGrid(bool show);

	signals:
		void scaleChanged(int factor);

	protected:
        void drawForeground(QPainter *painter, const QRectF &rect);
		void wheelEvent(QWheelEvent *event);

	private:
		QGraphicsScene *scene;
		QGraphicsPixmapItem *image_item;
		QGraphicsTextItem *text_item;

		QPixmap image_pixmap;
		bool grid_visible;
		int grid_interval;
        QPen grid_pen;
		//bool mouse_scroll;
		float scale_factor;
};

#endif // CONTROL_PANEL_IMAGE_VIEWER_H
