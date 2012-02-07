/******************************************************************************
 * battery_display.h
 *
 * Author:      Matt Richard
 * Date:        July 15, 2011
 * Description: 
 *****************************************************************************/

#ifndef CONTROL_PANEL_BATTERY_DISPLAY_H
#define CONTROL_PANEL_BATTERY_DISPLAY_H

#include <QGraphicsView>

QT_BEGIN_NAMESPACE
class QGraphicsRectItem;
class QGraphicsPixmapItem;
class QGraphicsTextItem;
class QGraphicsScene;
QT_END_NAMESPACE


class BatteryDisplay : public QGraphicsView
{
    public:
        BatteryDisplay(QWidget *parent = 0);
        int getBatteryLevel() const;
        void setBatteryLevel(float level);

    private:
        void updateBatteryDisplay();

        QGraphicsScene *scene;
        QGraphicsRectItem *battery_level_item;
        QGraphicsPixmapItem *battery_background_item;
        QGraphicsPixmapItem *battery_foreground_item;
        QGraphicsTextItem *battery_text_item;
        float battery_level;
};

#endif // CONTROL_PANEL_BATTERY_DISPLAY_H
