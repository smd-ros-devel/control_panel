/******************************************************************************
 * image_viewer.cpp
 *
 * Author:      Matt Richard
 * Date:        Jan 4, 2012
 * Description: A graphics view widget for displaying a robots video, LiDAR,
 *              and map feeds. The view allows for a grip to be overlayed on
 *              top of the image, zooming in and out, and moving the image
 *              around.
 *****************************************************************************/

#include <QtGui>
#include "control_panel/widgets/image_viewer.h"

ImageViewer::ImageViewer(QWidget *parent) : QGraphicsView(parent)
{
	grid_visible = false;
    grid_interval = 20;
    grid_pen.setColor(Qt::blue);
    grid_pen.setWidth(1);
//	mouse_scroll = true;
	scale_factor = 1.0;

	image_item = new QGraphicsPixmapItem(image_pixmap);
	image_item->setShapeMode(QGraphicsPixmapItem::BoundingRectShape);


	QFont font;
	font.setPointSize(18);

	text_item = new QGraphicsTextItem(tr("Loading..."));
	text_item->setFont(font);
	text_item->setDefaultTextColor(Qt::white);

	scene = new QGraphicsScene(this);
	scene->setBackgroundBrush(Qt::black);
	scene->addItem(text_item);
	scene->addItem(image_item);

	setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	setRenderHints(QPainter::Antialiasing | 
		QPainter::SmoothPixmapTransform);
	setFrameStyle(QFrame::NoFrame | QFrame::Plain);
	setScene(scene);
	setDragMode(QGraphicsView::ScrollHandDrag);
	setResizeAnchor(QGraphicsView::AnchorViewCenter);
    setTransformationAnchor(QGraphicsView::AnchorViewCenter);
}

QPixmap ImageViewer::getImagePixmap() const
{
	return image_pixmap;
}

bool ImageViewer::gridVisible() const
{
	return grid_visible;
}

void ImageViewer::setGridLineInterval(int pixels)
{
    if(pixels > 0)
	    grid_interval = pixels;
}

void ImageViewer::setImagePixmap(const QPixmap &pixmap, int interval)
{
    if(interval > 0)
        grid_interval = interval;

	image_pixmap = pixmap;

	image_item->setPixmap(image_pixmap);

    scene->setSceneRect(scene->itemsBoundingRect());
}

void ImageViewer::setImageVisible(bool visible)
{
	if(visible)
		image_pixmap.fill(Qt::transparent);
	else
		image_pixmap.fill(Qt::black);
	
	text_item->setVisible(visible);
	text_item->setPos((scene->width() / 2.0) - (text_item->boundingRect().width() / 2.0),
		(scene->height() / 2.0) - (text_item->boundingRect().height() / 2.0));

	image_item->setPixmap(image_pixmap);
    image_item->setVisible(visible);
}

void ImageViewer::setScale(int factor)
{
	scale_factor = (float)factor / 100.0;

    resetMatrix();
	//resetTransform();

	//if(mouse_scroll)
	//	setResizeAnchor(QGraphicsView::AnchorUnderMouse);
	//else
	//	setResizeAnchor(QGraphicsView::AnchorViewCenter);

	//mouse_scroll = false;

	scale(scale_factor, scale_factor);
}

void ImageViewer::showGrid(bool show)
{
	grid_visible = show;
}

void ImageViewer::drawForeground(QPainter *painter, const QRectF &rect)
{
    if(grid_visible)
    {
        double left = (int)rect.left() - ((int)rect.left() % grid_interval);
        double top = (int)rect.top() - ((int)rect.top() % grid_interval) +
            ((int)scene->height() % grid_interval);

        QVarLengthArray<QLineF, 100> lines_x;
        for(double x = left; x < rect.right(); x += grid_interval)
            lines_x.append(QLineF(x, rect.top(), x, rect.bottom() - 1));

        QVarLengthArray<QLineF, 100> lines_y;
        for(double y = top; y < rect.bottom(); y += grid_interval)
            lines_y.append(QLineF(rect.left(), y, rect.right(), y));

        painter->setPen(grid_pen);
        painter->drawLines(lines_x.data(), lines_x.size());
        painter->drawLines(lines_y.data(), lines_y.size());
    }
}

void ImageViewer::wheelEvent(QWheelEvent *event)
{
//	centerOn(mapToScene(event->pos()));
//	mouse_scroll = true;

	scale_factor += (float)event->delta() / 1000.0;

	if(scale_factor < 0.25)
		scale_factor = 0.25;
	else if(scale_factor > 4.0)
		scale_factor = 4.0;

	emit scaleChanged((int)(scale_factor * 100));

	event->accept();
}
