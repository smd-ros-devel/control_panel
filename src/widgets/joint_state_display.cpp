/* @todo Add license here */

/**
 * \file   joint_state_display.cpp
 * \date   Feb 8, 2012
 * \author Matt Eichard
 */
#include <QtGui>
#include "control_panel/widgets/joint_state_display.h"

JointStateDisplay::JointStateDisplay(QWidget *parent)
    : QWidget(parent) // NOTE: This constructor makes the widget useless. DO NOT USE
{
    joint_name_header = "Joints";
    use_position = false;
    use_velocity = false;
    use_effort = false;
}

JointStateDisplay::JointStateDisplay(const QStringList &names,
    const QStringList &display_names, QWidget *parent) : QWidget(parent)
{
    joint_names = names;
    joint_display_names = display_names;

    joint_name_header = "Joints";
    use_position = true;
    use_velocity = true;
    use_effort = true;

    createWidget();
}

JointStateDisplay::JointStateDisplay(const QString &widget_name,
    const QStringList &names, const QStringList &display_names,
    bool show_pos, bool show_vel, bool show_eff, QWidget *parent)
    : QWidget(parent)
{
    joint_names = names;
    joint_display_names = display_names;

    if(widget_name == "")
        joint_name_header = "Joints";
    else
        joint_name_header = widget_name;

    use_position = show_pos;
    use_velocity = show_vel;
    use_effort = show_eff;

    createWidget();
}

void JointStateDisplay::zeroValues()
{
    position.clear();
    velocity.clear();
    effort.clear();
}

void JointStateDisplay::createWidget()
{
    int i;
    int rows = 0;
    int cols = 0;
    QLabel *temp_label;

    zeroValues();

    // Create grid layout
    QGridLayout *grid_layout = new QGridLayout;

    temp_label = new QLabel(joint_name_header);
    grid_layout->addWidget(temp_label, 0, 0, Qt::AlignHCenter);
    rows++;

    // Add all joint display names to the grid layout
    for(i = 0; i < joint_display_names.size(); i++)
    {
        temp_label = new QLabel(joint_display_names.at(i));
        grid_layout->addWidget(temp_label, rows, 0, Qt::AlignLeft);
        rows++;
    }
    cols++;

    // Create the joint position column
    if(use_position)
    {
        temp_label = new QLabel("Position");
        grid_layout->addWidget(temp_label, 0, cols, Qt::AlignHCenter);
        grid_layout->setColumnMinimumWidth(1, 100);

        position_labels = new QList<QLabel *>;
        for(i = 1; i < rows; i++)
        {
            temp_label = new QLabel;
            position_labels->push_back(temp_label); // Add label to list
            grid_layout->addWidget(temp_label, i, cols, Qt::AlignRight);
        }
        cols++;
    }

    // Create the joint velocity column
    if(use_velocity)
    {
        temp_label = new QLabel("Velocity");
        grid_layout->addWidget(temp_label, 0, 2, Qt::AlignHCenter);
        grid_layout->setColumnMinimumWidth(2, 100);

        velocity_labels = new QList<QLabel *>;
        for(i = 1; i < rows; i++)
        {
            temp_label = new QLabel;
            velocity_labels->push_back(temp_label);
            grid_layout->addWidget(temp_label, i, cols, Qt::AlignRight);
        }
        cols++;
    }

    // Create the joint effort column
    if(use_effort)
    {
        temp_label = new QLabel("Effort");
        grid_layout->addWidget(temp_label, 0, 3, Qt::AlignHCenter);
        grid_layout->setColumnMinimumWidth(3, 100);

        effort_labels = new QList<QLabel *>;
        for(i = 1; i < rows; i++)
        {
            temp_label = new QLabel;
            effort_labels->push_back(temp_label);
            grid_layout->addWidget(temp_label, i, cols, Qt::AlignRight);
        }
        cols++;
    }

    widget_layout = new QHBoxLayout;
    widget_layout->addStretch();
    widget_layout->addLayout(grid_layout);
    widget_layout->addStretch();

    setLayout(widget_layout);
}

void JointStateDisplay::updateJointStateDisplay(const QStringList &names,
    std::vector<double> pos, std::vector<double> vel)
{
    
}

void JointStateDisplay::updateJointStateDisplay(const QStringList &names,
    const std::vector<double> &pos, const std::vector<double> &vel, const std::vector<double> &eff)
{
    int index;

    for(int i = 0; i < names.size(); i++)
    {
        index = joint_names.indexOf(names.at(i));

        if(index != -1)
        {
            if(use_position)
                ((QLabel *)position_labels->at(index))->setText(QString("%1").arg(pos[index], 6, 'f', 1) + QString(" m"));

            if(use_velocity)
                ((QLabel *)velocity_labels->at(index))->setText(QString("%1").arg(vel[index], 6, 'f', 1) + QString(" rad/s"));

            if(use_effort)
                ((QLabel *)effort_labels->at(index))->setText(QString("%1").arg(eff[index], 6, 'f', 1) + QString(" rad/s^2"));
        }
    }
}
