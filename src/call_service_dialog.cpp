/* @todo Add license here */

/**
 * \file   call_service_dialog.cpp
 * \date   March 16, 2012
 * \author Matt Richard
 */
#include <QtGui>
#include "control_panel/call_service_dialog.h"

CallServiceDialog::CallServiceDialog(QWidget *parent) : QDialog(parent)
{
    createDialog();
}

CallServiceDialog::CallServiceDialog(const QStringList &services,
    QWidget *parent) : QDialog(parent)
{
    createDialog();

    service_list->addItems(services);
}

QString CallServiceDialog::getSelectedService() const
{
    if(service_list->currentItem() != 0)
        return service_list->currentItem()->text();
    return QString();
}

void CallServiceDialog::createDialog()
{
    service_list = new QListWidget;
    service_list->setSortingEnabled(true);
    connect(service_list, SIGNAL(itemDoubleClicked(QListWidgetItem *)),
            this, SLOT(accept()));

    button_box = new QDialogButtonBox(QDialogButtonBox::Cancel |
                                      QDialogButtonBox::Ok);
    connect(button_box, SIGNAL(accepted()), this, SLOT(accept()));
    connect(button_box, SIGNAL(rejected()), this, SLOT(reject()));

    QVBoxLayout *dialog_layout = new QVBoxLayout;
    dialog_layout->addWidget(service_list);
    dialog_layout->addWidget(button_box);
    setLayout(dialog_layout);

    setWindowTitle(tr("Call Service"));
}
