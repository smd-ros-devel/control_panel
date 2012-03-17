/* @todo Add license here */

/**
 * \file   call_service_dialog.h
 * \date   March 16, 2012
 * \author Matt Richard
 */
#ifndef CONTROL_PANEL_CALL_SERVICE_DIALOG_H
#define CONTROL_PANEL_CALL_SERVICE_DIALOG_H

#include <QDialog>
#include <QListWidget>

QT_BEGIN_NAMESPACE
class QDialogButtonBox;
class QString;
class QStringList;
QT_END_NAMESPACE

/**
 * \class CallServiceDialog
 * \brief Dialog that displays the list of services a robot provides and lets a user select one.
 */
class CallServiceDialog : public QDialog
{
    public:
        CallServiceDialog(QWidget *parent = 0);
        CallServiceDialog(const QStringList &services, QWidget *parent = 0);
        void addService(const QString &service) { service_list->addItem(service); }
        QString getSelectedService() const;

    private:
        void createDialog();
        QListWidget *service_list;
        QDialogButtonBox *button_box;
};

#endif // CONTROL_PANEL_CALL_SERVICE_DIALOG_H
