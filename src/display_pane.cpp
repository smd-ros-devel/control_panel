/*****************************************************************************
 * display_pane.cpp
 *
 * Author:      Matt Richard
 * Date:        Nov 9, 2011
 * Description: This is a base class that can be inherited to create 
 *              different robot data displays (e.g., raw data display or 
 *              processed data display).
 ****************************************************************************/

#include <QtGui>
#include "control_panel/display_pane.h"


/******************************************************************************
 * Function:    DisplayPane
 * Author:      Matt Richard
 * Parameters:  QWidget *parent - parent widget
 * Returns:     None
 * Description: Constructor.
 *****************************************************************************/
DisplayPane::DisplayPane(QWidget *parent)
    : QWidget(parent)
{
    image_off = true;
	source_selection_list << "Off";

    QFont font;
    font.setPointSize(18);

    title_label = new QLabel;
    title_label->setFont(font);


	image_viewer = new ImageViewer;
	image_viewer->setAlignment(Qt::AlignCenter);
	image_viewer->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
	image_viewer->setFrameStyle(QFrame::Panel | QFrame::Sunken);
	image_viewer->setMinimumSize(0, 240);


    source_label = new QLabel(tr("Source: "));
    source_label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

    source_selector = new QComboBox;
	source_selector->setSizeAdjustPolicy(QComboBox::AdjustToContents);

	scale_slider = new QSlider(Qt::Horizontal);
	scale_slider->setRange(25, 400);
//	scale_slider->setSingleStep(25);
	scale_slider->setTickPosition(QSlider::TicksBelow);
	scale_slider->setTickInterval(25);

	grid_checkbox = new QCheckBox(tr("Grid"));


    connect(source_selector, SIGNAL(currentIndexChanged(int)),
		this, SLOT(sourceChanged(int)));
	connect(scale_slider, SIGNAL(valueChanged(int)),
		image_viewer, SLOT(setScale(int)));
	connect(image_viewer, SIGNAL(scaleChanged(int)),
		scale_slider, SLOT(setValue(int)));
    connect(grid_checkbox, SIGNAL(toggled(bool)),
        image_viewer, SLOT(showGrid(bool)));

    // Create video pane layout
    QHBoxLayout *header_hlayout = new QHBoxLayout;
    header_hlayout->addWidget(title_label, 0, Qt::AlignLeft);
    header_hlayout->addStretch();
    header_hlayout->addWidget(source_label, 0, Qt::AlignRight);
    header_hlayout->addWidget(source_selector, 0, Qt::AlignLeft);


	QHBoxLayout *footer_hlayout = new QHBoxLayout;
	footer_hlayout->addWidget(scale_slider);
	footer_hlayout->addWidget(grid_checkbox, 0, Qt::AlignRight);


    display_pane_layout = new QVBoxLayout;
    display_pane_layout->addLayout(header_hlayout);
	display_pane_layout->addWidget(image_viewer);
	display_pane_layout->addLayout(footer_hlayout);

    setLayout(display_pane_layout);
}

/******************************************************************************
** Function:    addSource
** Author:      Matt Richard
** Parameters:  char *new_source - source string to add
** Returns:     void
** Description:
******************************************************************************/
void DisplayPane::addSource(const QString &new_source)
{
	source_selection_list << new_source;
}

/******************************************************************************
** Function:    addSourceList
** Author:      Matt Richard
** Parameters:  QStringList source_list - list of source strings to add
** Returns:     void
** Description:
******************************************************************************/
void DisplayPane::addSourceList(const QStringList &source_list)
{
	source_selection_list << source_list;
}

/******************************************************************************
** Function:    getTitle
** Author:      Matt Richard
** Parameters:  None
** Returns:     std::string - the display pane's title
** Description:
******************************************************************************/
std::string DisplayPane::getTitle() const
{
	return title.toStdString();
}

/******************************************************************************
** Function:    getImage
** Author:      Matt Richard
** Parameters:  None
** Returns:     QImage - current image
** Description:
******************************************************************************/
QImage DisplayPane::getImage() const
{
	return image;
}

/******************************************************************************
** Function:    getCurrentSource
** Author:      Matt Richard
** Parameters:  None
** Returns:     std::string - the current source string
** Description:
******************************************************************************/
std::string DisplayPane::getCurrentSource() const
{
	return source_selector->currentText().toStdString();
}

/******************************************************************************
** Function:
** Author:
** Parameters:
** Returns:
** Description:
******************************************************************************/
bool DisplayPane::isImageOff() const
{
	return image_off;
}

/******************************************************************************
** Function:    connectionStatusChanged
** Author:      Matt Richard
** Parameters:  int new_status - the new status
** Returns:     void
** Description: 
******************************************************************************/
void DisplayPane::connectionStatusChanged(int new_status)
{
    if(new_status == Globals::Connected)
    {
        source_selector->addItems(source_selection_list);

		image_off = false;
    }
    else if(new_status == Globals::Connecting) {}
    else if(new_status == Globals::Disconnected)
        source_selector->clear(); // clear source combobox
    else
        printf("Display pane received unknown status '%d'.\n", new_status);
}

/******************************************************************************
** Function:    setImage
** Author:      Matt Richard
** Parameters:  QImage new_image - the new image to display
** Returns:     void
** Description:
******************************************************************************/
void DisplayPane::setImage(const QImage &new_image, int grid_interval)
{
	// Check if the image display is off
	if(!image_off)
		image_viewer->setImagePixmap(QPixmap::fromImage(new_image),
            grid_interval);
}

/******************************************************************************
** Function:    setTitle
** Author:      Matt Richard
** Parameters:  char *new_title - the new display pane title
** Returns:     void
** Description:
******************************************************************************/
void DisplayPane::setTitle(const char *new_title)
{
	title = new_title;

	title_label->setText(title);
}

/******************************************************************************
** Function:    updateVideoDisplay
** Author:      Matt Richard
** Parameters:  int new_view - 
** Returns:     void
** Description: 
******************************************************************************/
void DisplayPane::sourceChanged(int index)
{
    if(index <= 0)
	{
        image_off = true;
		scale_slider->setValue(100);
		grid_checkbox->setChecked(false);
	}
    else
		image_off = false;

	image_viewer->setImageVisible(!image_off);
	scale_slider->setEnabled(!image_off);
	grid_checkbox->setEnabled(!image_off);

    emit changeSource(source_selector->currentText().toStdString());
}
