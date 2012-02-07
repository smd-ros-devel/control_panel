/*****************************************************************************
 * display_pane.h
 *
 * Author:      Matt Richard
 * Date:        Nov 9, 2011
 * Description: This is a base class that can be inherited to create 
 *              different robot data displays (e.g., raw data display or 
 *              processed data display).
 ****************************************************************************/

#ifndef CONTROL_PANEL_DISPLAY_PANE_H
#define CONTROL_PANEL_DISPLAY_PANE_H

#include <QWidget>

QT_BEGIN_NAMESPACE
class QCheckBox;
class QComboBox;
class QImage;
class QLabel;
class QSlider;
class QString;
class QStringList;
class QVBoxLayout;
QT_END_NAMESPACE

#include <stdio.h>
#include <string>

#include "widgets/image_viewer.h"
#include "globals.h"


class DisplayPane : public QWidget
{
    Q_OBJECT

    public:
        DisplayPane(QWidget *parent = 0);
		void addSource(const QString &new_source);
		void addSourceList(const QStringList &source_list);
		std::string getTitle() const;
		QImage getImage() const;
		std::string getCurrentSource() const;
		bool isImageOff() const;
		void setTitle(const char *new_title);

    public slots:
        void connectionStatusChanged(int new_status);
		void setImage(const QImage &new_image, int grid_interval = -1);
        void sourceChanged(int index);

    signals:
        void changeSource(const std::string source);

    private:
		bool image_off;
        QVBoxLayout *display_pane_layout;

        QLabel *title_label;
        QLabel *source_label;

        QComboBox *source_selector;
        QStringList source_selection_list;

		QImage image;
		QString title;

		ImageViewer *image_viewer;

		QSlider *scale_slider;

		QCheckBox *grid_checkbox;
};

#endif // CONTROL_PANEL_DISPLAY_PANE_H
