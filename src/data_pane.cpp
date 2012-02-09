/*****************************************************************************
** data_pane.cpp
**
** Author:      Matt Richard
** Date:        June 2011
** Description:
*****************************************************************************/

#include <QtGui>

#include "control_panel/data_pane.h"


/******************************************************************************
** Function:    DataPane
** Author:      Matt Richard
** Parameters:  QWidget *parent -
** Returns:     None
** Description: Constructor
******************************************************************************/
DataPane::DataPane(QWidget *parent)
    : QWidget(parent)
{
    gps_list = new QList<GpsDisplay *>;
    imu_list = new QList<ImuDisplay *>;
    joint_state_list = new QList<JointStateDisplay *>;
    odom_list = new QList<OdometryDisplay *>;

    use_range = false;

    createLabels();

    initializeLabels();

    createLayout();

    setLayout(data_pane_layout);
    setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
}

/******************************************************************************
** Function:    createLabels
** Author:      Matt Richard
** Parameters:  None
** Returns:     void
** Description: 
******************************************************************************/
void DataPane::createLabels()
{
    status_light = new QImage(":/images/status_lights/status_light_grey.png");

    status_light_label = new QLabel;
    status_light_label->setBackgroundRole(QPalette::Base);
    status_light_label->setPixmap(QPixmap::fromImage(status_light->scaled(QSize(30, 30),
                                  Qt::KeepAspectRatio, Qt::SmoothTransformation)));
    status_light_label->setAlignment(Qt::AlignCenter);


    QFont font;
    font.setPointSize(18);

    connection_status_label = new QLabel;
    connection_status_label->setFont(font);
    connection_status_label->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);


    rc_mode_label = new QLabel("RC Mode: Disabled");

    range_label = new QLabel("");
    //battery_status_label = new QLabel;

/*
    heading_label = new QLabel;
    roll_label = new QLabel;
    pitch_label = new QLabel;
    yaw_label = new QLabel;
    altitude_label = new QLabel;
    attitude_label = new QLabel;


    wheelFR_label = new QLabel;
    wheelFL_label = new QLabel;
    wheelRR_label = new QLabel;
    wheelRL_label = new QLabel;


    speed_label = new QLabel;
    turn_label = new QLabel;
    slip_label = new QLabel;
    slide_label = new QLabel;


    gps_label = new QLabel;
    radiation_label = new QLabel;
    temp_label = new QLabel;
    light_label = new QLabel;
    pressure_label = new QLabel;
*/
}

/******************************************************************************
** Function:    initializeLabels
** Author:      Matt Richard
** Parameters:  None
** Returns:     void
** Description: 
******************************************************************************/
void DataPane::initializeLabels()
{
    connection_status_label->setText(tr("Disconnected"));
//    connection_status_label->setMinimumWidth(strlen("Disconnected"));
    //battery_status_label->setText(tr("Battery: "));

/*
    heading_label->setText(tr("Heading"));
    roll_label->setText(tr("Roll:        "));
    pitch_label->setText(tr("Pitch:       "));
    yaw_label->setText(tr("Yaw:         "));
    altitude_label->setText(tr("Altitude:      "));
    attitude_label->setText(tr("Attitude"));


    wheelFR_label->setText(tr("Wheel FR: "));
    wheelFL_label->setText(tr("Wheel FL: "));
    wheelRR_label->setText(tr("Wheel RR: "));
    wheelRL_label->setText(tr("Wheel RL: "));


    speed_label->setText(tr("Speed: "));
    turn_label->setText(tr("Turn: "));
    slip_label->setText(tr("Slip: "));
    slide_label->setText(tr("Slide: "));


    gps_label->setText(tr("GPS: "));
    radiation_label->setText(tr("Radiation: "));
    temp_label->setText(tr("Temperature: "));
    light_label->setText(tr("Light: "));
    pressure_label->setText(tr("Pressure: "));
*/
}

/******************************************************************************
** Function:    createLayout
** Author:      Matt Richard
** Parameters:  None
** Returns:     void
** Description: 
******************************************************************************/
void DataPane::createLayout()
{
    // Connection column layout
    QHBoxLayout *connection_hlayout = new QHBoxLayout;
    connection_hlayout->addWidget(status_light_label, 0, Qt::AlignLeft);
    connection_hlayout->addWidget(connection_status_label, 0, Qt::AlignLeft);
    //connection_hlayout->addStretch();

    //takeoff_land_button = new QPushButton("Takeoff");
    //connect(takeoff_land_button, SIGNAL(clicked()),
    //    this, SLOT(takeoffLandButtonClicked()));

    battery_display = new BatteryDisplay();

    QVBoxLayout *connection_vlayout = new QVBoxLayout;
    connection_vlayout->addLayout(connection_hlayout);
    connection_vlayout->addWidget(rc_mode_label);
    connection_vlayout->addStretch();
    connection_vlayout->addWidget(range_label);
//    connection_vlayout->addWidget(takeoff_land_button, 0, Qt::AlignLeft);
    connection_vlayout->addStretch();
    connection_vlayout->addWidget(battery_display, 0, Qt::AlignLeft);

/*
    // Heading indicator column layout
    heading_indicator = new HeadingIndicator();

    QVBoxLayout *heading_vlayout = new QVBoxLayout;
    heading_vlayout->addWidget(heading_label, 0, Qt::AlignCenter);
    heading_vlayout->addWidget(heading_indicator);
    heading_vlayout->addStretch();


    // Roll, pitch, yaw, and altitude column layout
    QVBoxLayout *roll_pitch_yaw_vlayout = new QVBoxLayout;
    roll_pitch_yaw_vlayout->addStretch();
    roll_pitch_yaw_vlayout->addWidget(roll_label, 0, Qt::AlignLeft);
    roll_pitch_yaw_vlayout->addWidget(pitch_label, 0, Qt::AlignLeft);
    roll_pitch_yaw_vlayout->addWidget(yaw_label, 0, Qt::AlignLeft);
    roll_pitch_yaw_vlayout->addWidget(altitude_label, 0, Qt::AlignLeft);
    roll_pitch_yaw_vlayout->addStretch();


    // Attitude indicator column layout
    attitude_indicator = new AttitudeIndicator();

    QVBoxLayout *attitude_vlayout = new QVBoxLayout;
    attitude_vlayout->addWidget(attitude_label, 0 , Qt::AlignCenter);
    attitude_vlayout->addWidget(attitude_indicator);
    attitude_vlayout->addStretch();


    // Wheel rpms column layout
    QVBoxLayout *wheel_vlayout = new QVBoxLayout;
    wheel_vlayout->addWidget(wheelFR_label);
    wheel_vlayout->addWidget(wheelFL_label);
    wheel_vlayout->addWidget(wheelRL_label);
    wheel_vlayout->addWidget(wheelRR_label);
    wheel_vlayout->addStretch();


    // Speed, turn, slip, and slide column layout
    QVBoxLayout *speed_vlayout = new QVBoxLayout;
    speed_vlayout->addWidget(speed_label);
    speed_vlayout->addWidget(turn_label);
    speed_vlayout->addWidget(slip_label);
    speed_vlayout->addWidget(slide_label);
    speed_vlayout->addStretch();


    // GPS, radiation, temperature, light, and pressure column layout
    QVBoxLayout *gps_vlayout = new QVBoxLayout;
    gps_vlayout->addWidget(gps_label);
    gps_vlayout->addWidget(radiation_label);
    gps_vlayout->addWidget(temp_label);
    gps_vlayout->addWidget(light_label);
    gps_vlayout->addWidget(pressure_label);
    gps_vlayout->addStretch();
*/

    // Data pane final layout
    data_pane_layout = new QHBoxLayout;
    //data_pane_layout->setSpacing(0);
    data_pane_layout->addLayout(connection_vlayout);
    //data_pane_layout->addStretch();
    //data_pane_layout->addSpacing(10);
//    data_pane_layout->setSpacing(0);

/*
    data_pane_layout->addStretch();
    data_pane_layout->addLayout(heading_vlayout);
    data_pane_layout->addSpacing(10);
    data_pane_layout->addStretch();
    data_pane_layout->addLayout(roll_pitch_yaw_vlayout);
    data_pane_layout->addSpacing(10);
    data_pane_layout->addStretch();
    data_pane_layout->addLayout(attitude_vlayout);
    data_pane_layout->addSpacing(20);
    data_pane_layout->addStretch();
    data_pane_layout->addLayout(wheel_vlayout);
    data_pane_layout->addSpacing(50);
    data_pane_layout->addStretch();
    data_pane_layout->addLayout(speed_vlayout);
    data_pane_layout->addSpacing(50);
    data_pane_layout->addStretch();
    data_pane_layout->addLayout(gps_vlayout);
    data_pane_layout->addSpacing(50);
    data_pane_layout->addStretch();
*/
}


/******************************************************************************
**                                   SLOTS
******************************************************************************/

void DataPane::takeoffLandButtonClicked()
{
    if(takeoff_land_button->text() == "Takeoff")
    {
        takeoff_land_button->setText("Land");
        emit takeoff();
    }
    else // land
    {
        takeoff_land_button->setText("Takeoff");
        emit land();
    }
}


/******************************************************************************
** Function:    connectionStatusChanged
** Author:      Matt Richard
** Parameters:  int new_status -
** Returns:     void
** Description: 
******************************************************************************/
void DataPane::connectionStatusChanged(int new_status)
{
    if(new_status == Globals::Connected)
    {
        // Load green status light image
        status_light->load(":/images/status_lights/status_light_green.png");
        status_light_label->setPixmap(
            QPixmap::fromImage(status_light->scaled(QSize(30, 30),
            Qt::KeepAspectRatio, Qt::SmoothTransformation)));

        connection_status_label->setText(tr("Connected    "));
    }
    else if(new_status == Globals::Connecting)
    {
        status_light->load(":/image/status_lights/status_light_yellow.png");
        status_light_label->setPixmap(QPixmap::fromImage(status_light->scaled(
            QSize(30, 30), Qt::KeepAspectRatio, Qt::SmoothTransformation)));

        connection_status_label->setText(tr("Connecting  "));
    }
    else if(new_status == Globals::Disconnected)
    {
        // Load grey status light image
        status_light->load(":/images/status_lights/status_light_grey.png");
        status_light_label->setPixmap(
            QPixmap::fromImage(status_light->scaled(QSize(30, 30),
            Qt::KeepAspectRatio, Qt::SmoothTransformation)));

        connection_status_label->setText(tr("Disconnected"));

        // Reset labels
        //initializeLabels();

        // Reset graphical displays
        //battery_display->setBatteryLevel(0);
        //heading_indicator->setYaw(0);
        //attitude_indicator->setAttitude(0, 0);
    }
    else
        printf("ERROR -- DataPane class received an unknown new status '%d'.\n", new_status);
}

/******************************************************************************
** Function:    updateImuData
** Author:      Matt Richard
** Parameters:  QQuaternion *imu_data - 
** Returns:     void
** Description: 
******************************************************************************/
/*
void DataPane::updateImuData(const QQuaternion &imu_data)
{
    double x = imu_data.x(),
           y = imu_data.y(),
           z = imu_data.z(),
           w = imu_data.scalar();


	// Calculate roll, pitch, and yaw
	double roll = atan2(2*(y*z + w*x), w*w - x*x - y*y + z*z) * (180.000 / PI);
	double pitch = asin(-2*(x*z - w*y)) * (180.000 / PI);
	double yaw = atan2(2*(x*y + w*z), w*w + x*x - y*y - z*z) * (180.000 / PI);


    // Display roll, pitch, and yaw
    roll_label->setText(tr("Roll:  %1").arg((int)roll, 5, 10, QLatin1Char(' '))
        + Globals::DegreesSymbol);
    pitch_label->setText(tr("Pitch: %1").arg(
        (int)pitch, 5, 10, QLatin1Char(' ')) + Globals::DegreesSymbol);
    yaw_label->setText(tr("Yaw:   %1").arg((int)yaw, 5, 10, QLatin1Char(' '))
        + Globals::DegreesSymbol);


    // Update graphics displays
    heading_indicator->setYaw(yaw);
    attitude_indicator->setAttitude(roll, pitch);
}
*/

/******************************************************************************
** Function:    updateGpsData
** Author:      Matt Richard
** Parameters:  double latitude - 
**              double longitude -
**              double altitude -
** Returns:     void
** Description: 
******************************************************************************/
/*
void DataPane::updateGpsData(double latitude, double longitude,
                             double altitude)
{
    QString lat_str = tr("GPS: %1").arg(
        fabs(latitude), 6, 'g', 6, QLatin1Char(' ')) + Globals::DegreesSymbol;
    QString long_str = tr(", %1").arg(
        fabs(longitude), 6, 'g', 6, QLatin1Char(' ')) + Globals::DegreesSymbol;


    if(latitude >= 0)
        lat_str += "N";
    else
        lat_str += "S";

    if(longitude >= 0)
        long_str += "E";
    else
        long_str += "W";


    gps_label->setText(lat_str + long_str);


    altitude_label->setText(tr("Altitude: %1'").arg(altitude, 6, 'g', 6, QLatin1Char(' ')));
}
*/

/******************************************************************************
** Function:    updateBatteryData
** Author:      Matt Richard
** Parameters:  int battery_data - 
** Returns:     void
** Description: 
******************************************************************************/
void DataPane::updateBatteryData(float battery_data)
{
    battery_display->setBatteryLevel(battery_data);

    //battery_status_label->setText(tr("Battery: %1%").arg(battery_data));
}

/******************************************************************************
** Function:    updateWheelData
** Author:      Matt Richard
** Parameters:  double wheelFR - 
**              double wheelFL -
**              double wheelRR -
**              double wheelRL -
** Returns:     void
** Description: 
******************************************************************************/
/*
void DataPane::updateWheelData(double wheelFR, double wheelFL,
                               double wheelRR, double wheelRL)
{
    wheelFR_label->setText(tr("Wheel FR: %1rpm").arg(wheelFR));
    wheelFL_label->setText(tr("Wheel FL: %1rpm").arg(wheelFL));
    wheelRR_label->setText(tr("Wheel RR: %1rpm").arg(wheelRR));
    wheelRL_label->setText(tr("Wheel RL: %1rpm").arg(wheelRL));
}
*/


GpsDisplay *DataPane::addGpsDisplay(const QString &name, bool lat, bool lon,
    bool alt)
{
    gps_display = new GpsDisplay(name, lat, lon, alt);

//    data_pane_layout->addStretch();
    //data_pane_layout->removeItem(data_pane_layout->itemAt(data_pane_layout->count()));
    data_pane_layout->addWidget(gps_display);
    //data_pane_layout->addStretch();

    gps_list->append(gps_display);

    return gps_display;
}

ImuDisplay *DataPane::addImuDisplay(const QString &name, bool roll,
    bool pitch, bool yaw, bool ang_vel, bool lin_accel, bool heading_graphic,
    bool attitude_graphic)
{
    imu_display = new ImuDisplay(name, roll, pitch, yaw, ang_vel, lin_accel,
        heading_graphic, attitude_graphic);

    data_pane_layout->addWidget(imu_display);

    imu_list->append(imu_display);

    return imu_display;
}

JointStateDisplay *DataPane::addJointStateDisplay(const QString &widget_name,
    const QStringList &names, const QStringList &display_names, bool show_pos,
    bool show_vel, bool show_eff)
{
    joint_state_display = new JointStateDisplay(widget_name, names,
        display_names, show_pos, show_vel, show_eff);

    data_pane_layout->addWidget(joint_state_display);

    joint_state_list->append(joint_state_display);

    return joint_state_display;
}

OdometryDisplay *DataPane::addOdometryDisplay(const QString &name, bool pos,
    bool rpy, bool lin_vel, bool ang_vel, bool heading_graphic,
    bool attitude_graphic)
{
    odom_display = new OdometryDisplay(name, pos, rpy, lin_vel, ang_vel,
        heading_graphic, attitude_graphic);

    data_pane_layout->addWidget(odom_display);

    odom_list->append(odom_display);

    return odom_display;
}


void DataPane::setRCModeText(const QString &mode)
{
    rc_mode_label->setText(QString("RC Mode: ") + mode);
}

void DataPane::showRangeLabel(bool show)
{
    range_label->setText("");

    use_range = show;

    if(use_range)
        range_label->setText("Height:");
}

void DataPane::updateRange(float range)
{
    if(use_range)
        range_label->setText(QString("Height: %1 m").arg(range, 0, 'g', 3));
}
