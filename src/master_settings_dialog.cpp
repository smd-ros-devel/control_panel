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
 * \file   master_settings_dialog.cpp
 * \date   Feb 3, 2012
 * \author Matt Richard

 * \brief  Qt dialog for allowing ROS_MASTER_URI and HOST_IP reconfiguration at runtime.
 */
#include <QtGui>
#include "control_panel/master_settings_dialog.h"


MasterSettingsDialog::MasterSettingsDialog(QWidget *parent) : QDialog(parent)
{
    disconnect_button_pushed = false;

    // Set up widgets
    master_uri_label = new QLabel("ROS Master URI: ");
    master_uri_lineedit = new QLineEdit("http://localhost:11311");

    host_ip_label = new QLabel("Host IP: ");
    host_ip_lineedit = new QLineEdit;

    use_env_vars_checkbox = new QCheckBox("Use environmental variables");

    disconnect_button = new QPushButton("Disconnect");

    cancel_connect_buttonbox = new QDialogButtonBox(QDialogButtonBox::Cancel);
    cancel_connect_buttonbox->addButton("Connect", QDialogButtonBox::AcceptRole);



    // Connections
    connect(use_env_vars_checkbox, SIGNAL(clicked(bool)), SLOT(useEnvVarsChecked(bool)));
    connect(disconnect_button, SIGNAL(clicked()), SLOT(disconnectButtonPushed()));
    connect(cancel_connect_buttonbox, SIGNAL(accepted()), SLOT(accept()));
    connect(cancel_connect_buttonbox, SIGNAL(rejected()), SLOT(reject()));


    // Create and set layout
    dialog_layout = new QGridLayout;
    dialog_layout->addWidget(master_uri_label, 0, 0, Qt::AlignLeft);
    dialog_layout->addWidget(master_uri_lineedit, 0, 1);
    dialog_layout->addWidget(host_ip_label, 1, 0, Qt::AlignLeft);
    dialog_layout->addWidget(host_ip_lineedit, 1, 1);
    dialog_layout->addWidget(use_env_vars_checkbox, 2, 1, Qt::AlignRight);
    dialog_layout->addWidget(disconnect_button, 3, 0, Qt::AlignLeft);
    dialog_layout->addWidget(cancel_connect_buttonbox, 3, 1, Qt::AlignRight);
    setLayout(dialog_layout);
    layout()->setSizeConstraint(QLayout::SetFixedSize);

    setWindowTitle("ROS Master Settings");
}

std::string MasterSettingsDialog::getMasterURI() const
{
    return master_uri_lineedit->text().toStdString();
}

std::string MasterSettingsDialog::getHostIP() const
{
    return host_ip_lineedit->text().toStdString();
}

void MasterSettingsDialog::setMasterURI(const QString &text)
{
    master_uri_lineedit->setText(text);
}

void MasterSettingsDialog::setHostIP(const QString &text)
{
    host_ip_lineedit->setText(text);
}

bool MasterSettingsDialog::useEnvironmentalVariables() const
{
    return use_env_vars_checkbox->isChecked();
}

void MasterSettingsDialog::useEnvVarsChecked(bool checked)
{
    // Toggle the QLineEdits' enabled property
    master_uri_lineedit->setEnabled(!checked);
    host_ip_lineedit->setEnabled(!checked);
}

void MasterSettingsDialog::disconnectButtonPushed()
{
    disconnect_button_pushed = true;

    accept();
}
