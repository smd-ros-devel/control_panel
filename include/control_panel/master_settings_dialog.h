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
 * \file   master_settings_dialog.h
 * \date   Feb 3, 2012
 * \author Matt Richard
 * \brief  Qt dialog for allowing ROS_MASTER_URI and HOST_IP reconfiguration at runtime.
 */
#ifndef CONTROL_PANEL_MASTER_SETTINGS_DIALOG_H
#define CONTROL_PANEL_MASTER_SETTINGS_DIALOG_H

#include <QDialog>

QT_BEGIN_NAMESPACE
class QCheckBox;
class QDialogButtonBox;
class QLabel;
class QLineEdit;
class QGridLayout;
class QPushButton;
QT_END_NAMESPACE

#include <string>

/**
 * \class  MasterSettingsDialog
 * \brief  A Qt dialog for configuring the Master URI and Host IP settings.
 */
class MasterSettingsDialog : public QDialog
{
    Q_OBJECT

    public:
        MasterSettingsDialog(QWidget *parent = 0);

        /**
         *
         */
        bool disconnectPushed() const { return disconnect_button_pushed; }

        /**
         * \brief Returns the Master URI the user entered.
         */
        std::string getMasterURI() const;

        /**
         * \brief Returns the Host IP the user entered
         */
        std::string getHostIP() const;

        /**
         * \brief Sets the master_uri_lineedit's contents to text.
         */
        void setMasterURI(const QString &text);

        /**
         * \brief Sets the host_ip_lineedit's contents to text.
         */
        void setHostIP(const QString &text);

        /**
         * \brief Returns true if the use_env_vars_checkbox is checked.
         */
        bool useEnvironmentalVariables() const;


    private slots:
        /**
         * \brief Slot that toggles the QLineEdits' enabled property
         *
         * \param checked The state of the checkbox
         */
        void useEnvVarsChecked(bool checked);

        void disconnectButtonPushed();

    private:
        QGridLayout *dialog_layout;

        QLabel *master_uri_label;
        QLabel *host_ip_label;

        QLineEdit *master_uri_lineedit;
        QLineEdit *host_ip_lineedit;

        QCheckBox *use_env_vars_checkbox;

        QPushButton *disconnect_button;
        QDialogButtonBox *cancel_connect_buttonbox;

        bool disconnect_button_pushed;
};

#endif // CONTROL_PANEL_MASTER_SETTINGS_DIALOG_H
