/**
 * \file   robot_config.cpp
 * \date   Dec 5, 2011
 * \author Scott K Logan, Matt Richard
 */
#include "control_panel/robot_config.h"
#include <iostream>

RobotConfig::RobotConfig()
{
	defaults();
}

void RobotConfig::defaults()
{
	configFilePath = "";
	robotName = "unknown_robot";
	system = "Unknown System";
	driveSystem = "UnknownDriveSystem";
	imageFilePath = ":/images/unknown.jpg";
	image.load(imageFilePath);
    nameSpace = "unknown_namespace";
	sensors.defaults();
    joint_states.defaults();
    processedData.defaults();
	diagnostics.defaults();
	commands.defaults();
	controls.defaults();
}

QString RobotConfig::getRobotName()
{
	if(robotName.isEmpty())
		return QString("(unnamed robot)");
	return robotName;
}

int RobotConfig::loadFrom(QFile *file, bool getComponents)
{
	defaults();

	QDomDocument doc(file->fileName());
	if(file->error() != QFile::NoError)
	{
		std::cerr << "ERROR: Could not setup DOM Doc" << std::endl;
		return -1; // File Error
	}

	doc.setContent(file);
	QDomElement root = doc.documentElement();
	if( root.tagName() != "robot" )
		return -2; // Syntax Error

	configFilePath = file->fileName();
	QDomNode n = root.firstChild();
	while(!n.isNull())
	{
		QDomElement e = n.toElement(); // try to convert the node to an element.
		processElement(e, getComponents);
		n = n.nextSibling();
	}

	return 0; // OK
}

void RobotConfig::processElement(QDomElement e, bool getComponents)
{
	if(e.tagName() == "robotName")
		robotName = e.text();
	else if(e.tagName() == "system")
		system = e.text();
	else if(e.tagName() == "driveSystem")
		driveSystem = e.text();
	else if(e.tagName() == "imageFile")
	{
		if(!e.isNull() && !e.text().isEmpty())
		{
			imageFilePath = e.text();
			if(imageFilePath[0] != '/')
				imageFilePath.prepend(":images/");
			image.load(imageFilePath);
		}
	}
    else if(e.tagName() == "nameSpace")
        nameSpace = e.text();
	else if(e.tagName() == "sensors")
	{
		if(!e.isNull() && !e.text().isEmpty() && getComponents)
			processSensors(e);
	}
    else if(e.tagName() == "joints")
    {
        if(!e.isNull() && !e.text().isEmpty() && getComponents)
            processJoints(e);
    }
    else if(e.tagName() == "processedData")
    {
        if(!e.isNull() && !e.text().isEmpty() && getComponents)
            processProcessedData(e);
    }
	else if(e.tagName() == "diagnostics")
	{
		if(!e.isNull() && !e.text().isEmpty() && getComponents)
			processDiagnostics(e);
	}
	else if(e.tagName() == "commands")
	{
		if(!e.isNull() && !e.text().isEmpty() && getComponents)
			processCommands(e);
	}
	else if(e.tagName() == "controls")
	{
		if(!e.isNull() && !e.text().isEmpty() && getComponents)
			processControls(e);
	}
	else
		std::cerr << "WARNING: Unknown tag " << e.tagName().toStdString() << std::endl;
}

void RobotConfig::processSensors(QDomElement e)
{
	QDomNode n = e.firstChild();
	while(!n.isNull())
	{
		e = n.toElement();
		if(e.tagName() == "camera")
			addCamera(e);
		else if(e.tagName() == "laser")
			addLaser(e);
		else if(e.tagName() == "gps")
			addGPS(e);
		else if(e.tagName() == "compass")
			addCompass(e);
		else if(e.tagName() == "imu")
			addIMU(e);
		else if(e.tagName() == "range")
			addRange(e);
		else
			std::cerr << "WARNING: Unknown sensor tag " << e.tagName().toStdString() << std::endl;
		n = n.nextSibling();
	}
}

void RobotConfig::addCamera(QDomElement e)
{
	struct RobotCamera *new_cam = new struct RobotCamera;
	QDomNode n = e.firstChild();
	while(!n.isNull())
	{
		e = n.toElement();
		if(e.tagName() == "name")
			new_cam->name = e.text();
		else if(e.tagName() == "topicName")
			new_cam->topicName = e.text();
		else
			std::cerr << "WARNING: Unknown sensor tag " << e.tagName().toStdString() << std::endl;
		n = n.nextSibling();
	}
	new_cam->next = sensors.cameras;
	sensors.cameras = new_cam;
}

void RobotConfig::addLaser(QDomElement e)
{
	struct RobotLaser *new_laser = new struct RobotLaser;
	QDomNode n = e.firstChild();
	while(!n.isNull())
	{
		e = n.toElement();
		if(e.tagName() == "name")
			new_laser->name = e.text();
		else if(e.tagName() == "topicName")
			new_laser->topicName = e.text();
		else
			std::cerr << "WARNING: Unknown sensor tag " << e.tagName().toStdString() << std::endl;
		n = n.nextSibling();
	}
	new_laser->next = sensors.lasers;
	sensors.lasers = new_laser;
}

void RobotConfig::addGPS(QDomElement e)
{
	struct RobotGPS *new_gps = new struct RobotGPS;
	QDomNode n = e.firstChild();
	while(!n.isNull())
	{
		e = n.toElement();
		if(e.tagName() == "name")
			new_gps->name = e.text();
		else if(e.tagName() == "topicName")
			new_gps->topicName = e.text();
		else if(e.tagName() == "longitude")
			new_gps->longitude = true;
		else if(e.tagName() == "latitude")
			new_gps->latitude = true;
		else if(e.tagName() == "altitude")
			new_gps->altitude = true;
		else if(e.tagName() == "positionCovariance")
			new_gps->covariance = true;
		else
			std::cerr << "WARNING: Unknown sensor tag " << e.tagName().toStdString() << std::endl;
		n = n.nextSibling();
	}
	new_gps->next = sensors.gps;
	sensors.gps = new_gps;
}

void RobotConfig::addCompass(QDomElement e)
{
	struct RobotCompass *new_compass = new struct RobotCompass;
	QDomNode n = e.firstChild();
	while(!n.isNull())
	{
		e = n.toElement();
		if(e.tagName() == "name")
			new_compass->name = e.text();
		else if(e.tagName() == "topicName")
			new_compass->topicName = e.text();
		else
			std::cerr << "WARNING: Unknown sensor tag " << e.tagName().toStdString() << std::endl;
		n = n.nextSibling();
	}
	new_compass->next = sensors.compass;
	sensors.compass = new_compass;
}

void RobotConfig::addIMU(QDomElement e)
{
	struct RobotIMU *new_imu = new struct RobotIMU;
	QDomNode n = e.firstChild();
	while(!n.isNull())
	{
		e = n.toElement();
		if(e.tagName() == "name")
			new_imu->name = e.text();
		else if(e.tagName() == "topicName")
			new_imu->topicName = e.text();
        else if(e.tagName() == "roll")
            new_imu->roll = true;
        else if(e.tagName() == "pitch")
            new_imu->pitch = true;
        else if(e.tagName() == "yaw")
            new_imu->yaw = true;
		else if(e.tagName() == "angularVelocity")
			new_imu->angularVelocity = true;
		else if(e.tagName() == "linearAcceleration")
			new_imu->linearAcceleration = true;
		else if(e.tagName() == "hideAttitude")
			new_imu->hideAttitude = true;
		else if(e.tagName() == "hideHeading")
			new_imu->hideHeading = true;
		else if(e.tagName() == "hideLabels")
			new_imu->hideLabels = true;
		else
			std::cerr << "WARNING: Unknown sensor tag " << e.tagName().toStdString() << std::endl;
		n = n.nextSibling();
	}
	new_imu->next = sensors.imu;
	sensors.imu = new_imu;
}

void RobotConfig::addRange(QDomElement e)
{
	struct RobotRange *new_range = new struct RobotRange;
	QDomNode n = e.firstChild();
	while(!n.isNull())
	{
		e = n.toElement();
		if(e.tagName() == "name")
			new_range->name = e.text();
		else if(e.tagName() == "topicName")
			new_range->topicName = e.text();
		else
			std::cerr << "WARNING: Unknown sensor tag " << e.tagName().toStdString() << std::endl;
		n = n.nextSibling();
	}
	new_range->next = sensors.range;
	sensors.range = new_range;
}
/////////////////////////// End Sensors /////////////////////////////



////////////////////////// Joints //////////////////////////
void RobotConfig::processJoints(QDomElement e)
{
    QDomNode n = e.firstChild();
    while(!n.isNull())
    {
        e = n.toElement();
        if(e.tagName() == "joint")
            addJoint(e);
        else if(e.tagName() == "topicName")
            joint_states.topicName = e.text();
        else if(e.tagName() == "position")
            joint_states.position = true;
        else if(e.tagName() == "velocity")
            joint_states.velocity = true;
        else if(e.tagName() == "effort")
            joint_states.effort = true;
        else
            std::cerr << "WARNING: Unknown joints tag " << e.tagName().toStdString() << std::endl;
        n = n.nextSibling();
    }
}

void RobotConfig::addJoint(QDomElement e)
{
    struct RobotJoint *new_joint = new struct RobotJoint;
    QDomNode n = e.firstChild();
    while(!n.isNull())
    {
        e = n.toElement();
        if(e.tagName() == "name")
            new_joint->name = e.text();
        else if(e.tagName() == "displayName")
            new_joint->displayName = e.text();
        else
            std::cerr << "WARNING: Unknown joint tag " << e.tagName().toStdString() << std::endl;
        n = n.nextSibling();
    }
    new_joint->next = joint_states.joints;
    joint_states.joints = new_joint;
    joint_states.used = true;
}
////////////////////////// End Joints //////////////////////////////


/////////////////////////// Processed Data ///////////////////////////
void RobotConfig::processProcessedData(QDomElement e)
{
    QDomNode n = e.firstChild();
    while(!n.isNull())
    {
        e = n.toElement();
        if(e.tagName() == "image")
            addImage(e);
        else if(e.tagName() == "map")
            addMap(e);
        else if(e.tagName() == "odometry")
            addOdometry(e);
        else
            std::cerr << "WARNING: Unknown processed data tag " << e.tagName().toStdString() << std::endl;
        n = n.nextSibling();
    }
}

void RobotConfig::addImage(QDomElement e)
{
    struct RobotCamera *new_image = new struct RobotCamera;
    QDomNode n = e.firstChild();
    while(!n.isNull())
    {
        e = n.toElement();
        if(e.tagName() == "name")
            new_image->name = e.text();
        else if(e.tagName() == "topicName")
            new_image->topicName = e.text();
        else
            std::cerr << "WARNING: Unknown processed data tag " << e.tagName().toStdString() << std::endl;
        n = n.nextSibling();
    }
    new_image->next = processedData.images;
    processedData.images = new_image;
}

void RobotConfig::addMap(QDomElement e)
{
    struct RobotMap *new_map = new struct RobotMap;
    QDomNode n = e.firstChild();
    while(!n.isNull())
    {
        e = n.toElement();
        if(e.tagName() == "name")
            new_map->name = e.text();
        else if(e.tagName() == "topicName")
            new_map->topicName = e.text();
        else
            std::cerr << "WARNING: Unknown processed data tag " << e.tagName().toStdString() << std::endl;
        n = n.nextSibling();
    }
    new_map->next = processedData.maps;
    processedData.maps = new_map;
}

void RobotConfig::addOdometry(QDomElement e)
{
    struct RobotOdometry *new_odometry = new struct RobotOdometry;
    QDomNode n = e.firstChild();
    while(!n.isNull())
    {
        e = n.toElement();
        if(e.tagName() == "name")
            new_odometry->name = e.text();
        else if(e.tagName() == "topicName")
            new_odometry->topicName = e.text();
        else if(e.tagName() == "position")
            new_odometry->position = true;
        else if(e.tagName() == "orientation")
            new_odometry->orientation = true;
        else if(e.tagName() == "linearVelocity")
            new_odometry->linearVelocity = true;
        else if(e.tagName() == "angularVelocity")
            new_odometry->angularVelocity = true;
        else if(e.tagName() == "hideAttitude")
            new_odometry->hideAttitude = true;
        else if(e.tagName() == "hideHeading")
            new_odometry->hideHeading = true;
        else if(e.tagName() == "hideLabels")
            new_odometry->hideLabels = true;
        else
            std::cerr << "WARNING: Unknown processed data tag " << e.tagName().toStdString() << std::endl;
        n = n.nextSibling();
    }
    new_odometry->next = processedData.odometry;
    processedData.odometry = new_odometry;
}
/////////////////////// End Processed Data //////////////////////


//////////////////////// Diagnostics /////////////////////////
void RobotConfig::processDiagnostics(QDomElement e)
{
	QDomNode n = e.firstChild();
	while(!n.isNull())
	{
		e = n.toElement();
		if(e.tagName() == "topicName")
			diagnostics.topicName = e.text();
		else if(e.tagName() == "temperature")
			addTemperature(e);
		else if(e.tagName() == "voltage")
			addVoltage(e);
		else if(e.tagName() == "batteryLevel")
			addBatteryLevel(e);
		else
			std::cerr << "WARNING: Unknown diagnostic tag " << e.tagName().toStdString() << std::endl;
		n = n.nextSibling();
	}
}

void RobotConfig::addTemperature(QDomElement e)
{
	struct RobotTemperature *new_temperature = new struct RobotTemperature;
	QDomNode n = e.firstChild();
	while(!n.isNull())
	{
		e = n.toElement();
		if(e.tagName() == "name")
			new_temperature->name = e.text();
		else if(e.tagName() == "minRange")
			new_temperature->minRange = e.text().toFloat();
		else if(e.tagName() == "maxRange")
			new_temperature->maxRange = e.text().toFloat();
		else if(e.tagName() == "units")
			new_temperature->units = e.text();
		else
			std::cerr << "WARNING: Unknown diagnostic tag " << e.tagName().toStdString() << std::endl;
		n = n.nextSibling();
	}
	new_temperature->next = diagnostics.temperature;
	diagnostics.temperature = new_temperature;
	diagnostics.used = true;
}

void RobotConfig::addVoltage(QDomElement e)
{
	struct RobotVoltage *new_voltage = new struct RobotVoltage;
	QDomNode n = e.firstChild();
	while(!n.isNull())
	{
		e = n.toElement();
		if(e.tagName() == "name")
			new_voltage->name = e.text();
		else if(e.tagName() == "minRange")
			new_voltage->minRange = e.text().toFloat();
		else if(e.tagName() == "maxRange")
			new_voltage->maxRange = e.text().toFloat();
		else if(e.tagName() == "operatingVoltage")
			new_voltage->operatingVoltage = e.text().toFloat();
		else
			std::cerr << "WARNING: Unknown diagnostic tag " << e.tagName().toStdString() << std::endl;
		n = n.nextSibling();
	}
	new_voltage->next = diagnostics.voltage;
	diagnostics.voltage = new_voltage;
	diagnostics.used = true;
}

void RobotConfig::addBatteryLevel(QDomElement e)
{
	struct RobotBatteryLevel *new_batteryLevel = new struct RobotBatteryLevel;
	QDomNode n = e.firstChild();
	while(!n.isNull())
	{
		e = n.toElement();
		if(e.tagName() == "name")
			new_batteryLevel->name = e.text();
		else if(e.tagName() == "max")
			new_batteryLevel->max = e.text().toFloat();
		else
			std::cerr << "WARNING: Unknown diagnostic tag " << e.tagName().toStdString() << std::endl;
		n = n.nextSibling();
	}
	new_batteryLevel->next = diagnostics.batteryLevel;
	diagnostics.batteryLevel = new_batteryLevel;
	diagnostics.used = true;
}
////////////////////////// End Diagnostics //////////////////////////

void RobotConfig::processCommands(QDomElement e)
{
	QDomNode n = e.firstChild();
	while(!n.isNull())
	{
		e = n.toElement();
		if(e.tagName() == "custom")
			addCommandCustom(e);
		else if(e.tagName() == "takeoff")
			addCommandCustom(e);
		else if(e.tagName() == "land")
			addCommandCustom(e);
		else
			std::cerr << "WARNING: Unknown command tag " << e.tagName().toStdString() << std::endl;
		n = n.nextSibling();
	}
}

void RobotConfig::addCommandCustom(QDomElement e)
{
	struct RobotCommandCustom *new_custom = new struct RobotCommandCustom;
	QDomNode n = e.firstChild();
	while(!n.isNull())
	{
		QDomElement ee = n.toElement();
		if(ee.tagName() == "name" && e.tagName() == "custom")
			new_custom->name = ee.text();
		else if(ee.tagName() == "topicName")
			new_custom->topicName = ee.text();
		else
			std::cerr << "WARNING: Unknown command tag " << e.tagName().toStdString() << std::endl;
		n = n.nextSibling();
	}
	if(e.tagName() == "takeoff" || e.tagName() == "land")
		new_custom->name = e.text();
	new_custom->next = commands.custom;
	commands.custom = new_custom;
	commands.used = true;
}

void RobotConfig::processControls(QDomElement e)
{
	QDomNode n = e.firstChild();
	while(!n.isNull())
	{
		e = n.toElement();
		if(e.tagName() == "drive")
			addDrive(e);
		else
			std::cerr << "WARNING: Unknown control tag " << e.tagName().toStdString() << std::endl;
		n = n.nextSibling();
	}
}

void RobotConfig::addDrive(QDomElement e)
{
	struct RobotDrive *new_drive = new struct RobotDrive;
	QDomNode n = e.firstChild();
	while(!n.isNull())
	{
		e = n.toElement();
		if(e.tagName() == "topicName")
			new_drive->topicName = e.text();
		else
			std::cerr << "WARNING: Unknown control tag " << e.tagName().toStdString() << std::endl;
		n = n.nextSibling();
	}
	new_drive->next = controls.drive;
	controls.drive = new_drive;
	controls.used = true;
}

