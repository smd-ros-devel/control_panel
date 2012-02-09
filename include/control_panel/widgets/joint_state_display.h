/* @todo Add license here */

/**
 * \file   joint_state_display.h
 * \date   Feb 8, 2012
 * \author Matt Richard
 */
#ifndef CONTROL_PANEL_JOINT_STATE_DISPLAY_H
#define CONTROL_PANEL_JOINT_STATE_DISPLAY_H

#include <QWidget>
#include <QString>
#include <QStringList>
#include <QList>

QT_BEGIN_NAMESPACE
class QHBoxLayout;
class QLabel;
QT_END_NAMESPACE

#include <vector>

/**
 * \class JointStateDisplay
 * \brief Displays the position, velocity, and effort of a robots' joints
 */
class JointStateDisplay : public QWidget
{
    Q_OBJECT

    public:
        JointStateDisplay(QWidget *parent = 0); // <-- DO NOT USE THIS CONSTRUCTOR
        JointStateDisplay(const QStringList &names, const QStringList &display_names,
            QWidget *parent = 0);
        JointStateDisplay(const QString &widget_name, const QStringList &names,
            const QStringList &display_names, bool show_pos = true,
            bool show_vel = true, bool show_eff = false, QWidget *parent = 0);
        void zeroValues();

    public slots:
        void updateJointStateDisplay(const QStringList &names,
            std::vector<double> pos, std::vector<double> vel);
        void updateJointStateDisplay(const QStringList &names,
            const std::vector<double> &pos, const std::vector<double> &vel,
            const std::vector<double> &eff);

    private:
        void createWidget();

        QHBoxLayout *widget_layout;

        QString joint_name_header;

        QStringList joint_names;
        QStringList joint_display_names;

        QList<QLabel *> *position_labels;
        QList<QLabel *> *velocity_labels;
        QList<QLabel *> *effort_labels;

        bool use_position;
        bool use_velocity;
        bool use_effort;

        std::vector<double> position;
        std::vector<double> velocity;
        std::vector<double> effort;
};

#endif // CONTROL_PANEL_JOINT_STATE_DISPLAY
