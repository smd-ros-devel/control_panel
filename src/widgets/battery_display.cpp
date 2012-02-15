/******************************************************************************
** battery_display.cpp
**
** Author:      Matt Richard
** Date:        July 15, 2011
** Description: 
******************************************************************************/

#include <QtGui>

#include "control_panel/widgets/battery_display.h"


/******************************************************************************
** Function:    BatteryDisplay
** Author:      Matt Richard
** Parameters:  QWidget *parent
** Returns:     None
** Description: Constructor
******************************************************************************/
BatteryDisplay::BatteryDisplay(QWidget *parent)
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
    battery_background_item = new QGraphicsPixmapItem(
        QPixmap(":/images/battery/battery_background.png"));


    battery_foreground_item = new QGraphicsPixmapItem(
        QPixmap(":/images/battery/battery.png"));


    QBrush brush(QColor(0, 255, 0));
    QPen pen(Qt::NoPen);

    battery_level_item = new QGraphicsRectItem;
    battery_level_item->setBrush(brush);
    battery_level_item->setPen(pen);

    battery_text_item = new QGraphicsTextItem;
    battery_text_item->setDefaultTextColor(Qt::white);

    // Create graphics scene
    scene = new QGraphicsScene(this);
//    scene->setSceneRect(0, 0, battery_pixmap.width(), battery_pixmap.height());
    scene->addItem(battery_background_item);
    scene->addItem(battery_level_item);
    scene->addItem(battery_text_item);
    scene->addItem(battery_foreground_item);
    scene->setSceneRect(scene->itemsBoundingRect());

    setScene(scene);
    //ensureVisible(battery_item, 0, 0);
    setFixedSize(scene->sceneRect().toRect().size());


    // Initialize battery level
    setBatteryLevel(0.0);
    //scale(0.75, 0.75);
}

/******************************************************************************
** Function:    getBatteryLevel
** Author:      Matt Richard
** Parameters:  None
** Returns:     int - current battery level
** Description: 
******************************************************************************/
int BatteryDisplay::getBatteryLevel() const
{
    return battery_level;
}

/******************************************************************************
** Function:    setBatteryLevel
** Author:      Matt Richard
** Parameters:  int level - 
** Returns:     void
** Description: 
******************************************************************************/
void BatteryDisplay::setBatteryLevel(float level)
{
    battery_level = level;

    updateBatteryDisplay();
}

/******************************************************************************
** Function:    updateBatteryDisplay
** Author:      Matt Richard
** Parameters:  None
** Returns:     void
** Description: 
******************************************************************************/
void BatteryDisplay::updateBatteryDisplay()
{
    battery_level_item->setRect(0, 0, 90 * (battery_level / 100.0) + 5,
        scene->height());

    battery_text_item->setPlainText(QString("%1%").arg(battery_level));
    battery_text_item->setPos((scene->width() / 2.0) -
        (battery_text_item->boundingRect().width() / 2.0),
        (scene->height() / 2.0) - (battery_text_item->boundingRect().height() / 2.0));
}
