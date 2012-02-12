/* @todo Add license here */

/**
 * \file   robot_config.h
 * \date   Dec 5, 2011
 * \author Scott K Logan, Matt Richard
 * \brief  RobotConfig is the set of data that defines a robot's physical and sensory configuration.
 */
#ifndef CONTROL_PANEL_ROBOT_CONFIG_H
#define CONTROL_PANEL_ROBOT_CONFIG_H

#include <vector>
#include <QObject>
#include <QFile>
#include <QDomDocument>
#include <QImage>


///////////////////////////// Sensors //////////////////////////////

struct RobotCamera
{
	public:
		/* stored data */
		QString name;
		QString topicName;

		/* constructor */
		RobotCamera() : name("Unknown Camera"), topicName("unknown_camera")
		{
		}
};

struct RobotLaser
{
	public:
		/* stored data */
		QString name;
		QString topicName;

		/* constructor */
		RobotLaser() : name("Unknown Laser"), topicName("unknown_laser")
		{
		}
};

struct RobotGPS
{
	public:
		/* stored data */
		QString name;
		QString topicName;
		bool longitude;
		bool latitude;
		bool altitude;
		bool covariance;

		/* constructor */
		RobotGPS() : name("Unknown GPS"), topicName("unknown_gps"), longitude(false), latitude(false), altitude(false), covariance(false)
		{
		}
};

struct RobotCompass
{
	public:
		/* stored data */
		QString name;
		QString topicName;

		/* constructor */
		RobotCompass() : name("Unknown Compass"), topicName("unknown_compass")
		{
		}
};

struct RobotIMU
{
	public:
		/* stored data */
		QString name;
		QString topicName;
		bool roll;
		bool pitch;
		bool yaw;
		bool angularVelocity;
		bool linearAcceleration;
		bool hideAttitude;
		bool hideHeading;
		bool hideLabels;

		/* constructor */
		RobotIMU() : name("Unknown IMU"), topicName("unknown_imu/data"), roll(false), pitch(false), yaw(false), angularVelocity(false), linearAcceleration(false), hideAttitude(false), hideHeading(false), hideLabels(false)
		{
		}
};

struct RobotRange
{
	public:
		/* stored data */
		QString name;
		QString topicName;

		/* constructor */
		RobotRange() : name("Unknown Range"), topicName("unknown_range")
		{
		}
};

struct RobotSensors
{
	public:
		/* sensor list */
		std::vector<struct RobotCamera> cameras;
		std::vector<struct RobotLaser> lasers;
		std::vector<struct RobotGPS> gps;
		std::vector<struct RobotCompass> compass;
		std::vector<struct RobotIMU> imu;
		std::vector<struct RobotRange> range;

		/* set all to defaults */
		void defaults()
		{
			cameras.clear();
			lasers.clear();
			gps.clear();
			compass.clear();
			imu.clear();
			range.clear();
		}
};

////////////////////// End Sensors //////////////////////////



//////////////////////// Joints ///////////////////////////////

/**
 * \struct RobotJoint
 * \brief  @todo Fill this out
 */
struct RobotJoint
{
    public:
        /* stored data */
        QString name;
        QString displayName;

        /**
         * \brief Contstructor. Initializes linked list and data
         */
        RobotJoint()
            : name("unknown_joint"),
              displayName("Unknown Joint") { }
};

/**
 * \struct RobotJoints
 * \brief  @todo Fill this out
 */
struct RobotJoints
{
    public:
        /* stored data */
        QString topicName;
        bool position;
        bool velocity;
        bool effort;
        bool used;

        /* Joints list */
        std::vector<struct RobotJoint> joints;

        /**
         * \brief Constructor. Initializes public members
         */
        RobotJoints()
            : topicName("joint_states"),
              position(false),
              velocity(false),
              effort(false),
              used(false) { }

        /**
         * \brief Destroys linked list
         */
        void defaults()
        {
            topicName = "joint_states";
            position = false;
            velocity = false;
            effort = false;
            joints.clear();
        }
};

///////////////////// End Joints ////////////////////////////



//////////////////// Processed Data /////////////////////////

/**
 * \struct RobotDisparityImage
 * \brief  @todo Fill this out
 */
struct RobotDisparityImage
{
    public:
        /* stored data */
        QString name;
        QString topicName;

        /**
         * \brief Constructor. Initilizes data.
         */
        RobotDisparityImage()
            : name("Unknown Disparity Image"),
              topicName("unknown_disparity_image") { }
};

/**
 * \struct RobotMap
 * \brief  @todo Fill this out
 */
struct RobotMap
{
    public:
        /* stored data */
        QString name;
        QString topicName;

        /**
         * \brief Constructor. Sets member values to default values.
         */
        RobotMap()
            : name("Unknown Map"),
              topicName("unknown_map") { }
};

/**
 * \struct RobotOdometry
 * \brief  @todo Fill this out
 */
struct RobotOdometry
{
    public:
        /* stored data */
        QString name;
        QString topicName;
        bool position;
        bool orientation;
        bool linearVelocity;
        bool angularVelocity;
        bool hideAttitude;
        bool hideHeading;
        bool hideLabels;

        /**
         * Constructor. Initializes structure members
         */
        RobotOdometry()
            : name("Unknown Odometry"),
              topicName("unknown_odometry"),
              position(false),
              orientation(false),
              linearVelocity(false),
              angularVelocity(false),
              hideAttitude(false),
              hideHeading(false),
              hideLabels(false) 
        {
        }
};

/**
 * \struct RobotProcessedData
 * \brief  @todo Fill this out.
 */
struct RobotProcessedData
{
    public:
        /* Processed data lists */
        std::vector <struct RobotCamera> images;
        std::vector <struct RobotDisparityImage> disparity_images;
        std::vector <struct RobotMap> maps;
        std::vector <struct RobotOdometry> odometry;

        /**
         * \brief Sets all struct members to their defaults
         */
        void defaults()
        {
            images.clear();
            disparity_images.clear();
            maps.clear();
            odometry.clear();
        }
};

///////////////////////// End Processed Data //////////////////////////


//////////////////////////// Diagnostics ////////////////////////////

struct RobotTemperature
{
	public:
		/* stored data */
		QString name;
		float minRange;
		float maxRange;
		QString units;

		/* constructor */
		RobotTemperature() : name("Unknown Temperature"), minRange(0), maxRange(0), units("Celcius")
		{
		}
};

struct RobotVoltage
{
	public:
		/* stored data */
		QString name;
		float minRange;
		float maxRange;
		float operatingVoltage;

		/* constructor */
		RobotVoltage() : name("Unknown Temperature"), minRange(0), maxRange(0), operatingVoltage(0)
		{
		}
};

struct RobotBatteryLevel
{
	public:
		/* stored data */
		QString name;
		float max;

		/* constructor */
		RobotBatteryLevel() : name("Unknown Battery Level"), max(0)
		{
		}
};

struct RobotDiagnostics
{
	public:
		/* stored data */
		QString topicName;
		bool used;

		/* diagnostics list */
		std::vector<struct RobotTemperature> temperature;
		std::vector<struct RobotVoltage> voltage;
		std::vector<struct RobotBatteryLevel> batteryLevel;

		/* constructor */
		RobotDiagnostics() : topicName("diagnostics"), used(false)
		{
		}

		/* set all to defaults */
		void defaults()
		{
			topicName = "diagnostics";
			used = false;
			temperature.clear();
			voltage.clear();
			batteryLevel.clear();
		}
};

///////////////////////////// End Diagnostics /////////////////////////////


/////////////////////////// Commands //////////////////////////////////

struct RobotCommandCustom
{
	public:
		/* stored data */
		QString name;
		QString topicName;

		/* constructor */
		RobotCommandCustom() : name("Unknown Temperature"), topicName("unknown_custom_command")
		{
		}
};

struct RobotCommands
{
	public:
		/* stored data */
		bool used;

		/* commands list */
		std::vector<struct RobotCommandCustom> custom;

		/* constructor */
		RobotCommands() : used(false)
		{
		}

		/* set all to defaults */
		void defaults()
		{
			used = false;
			custom.clear();
		}
};

////////////////////////// End Commands ///////////////////////////////


//////////////////////////// Controls ////////////////////////////////

struct RobotDrive
{
	public:
		/* stored data */
		QString topicName;

		/* constructor */
		RobotDrive() : topicName("unknown_drive")
		{
		}
};

struct RobotControls
{
	public:
		/* stored data */
		bool used;

		/* controls list */
		std::vector<struct RobotDrive> drive;

		/* constructor */
		RobotControls() : used(false)
		{
		}

		/* set all to defaults */
		void defaults()
		{
			used = false;
			drive.clear();
		}
};

////////////////////////// End Controls /////////////////////////////



struct RobotConfig : public QObject
{
	Q_OBJECT

	public:
		/* background data */
		QString configFilePath;

		/* general data */
		QString robotName;
		QString system;
		QString driveSystem;
		QString imageFilePath;
		QImage image;
        QString nameSpace;

		/* data */
		struct RobotSensors sensors;
        struct RobotJoints joint_states;
        struct RobotProcessedData processedData;
		struct RobotDiagnostics diagnostics;
		struct RobotCommands commands;
		struct RobotControls controls;

		/* constructor */
		RobotConfig();

		/* set all to defaults */
		void defaults();

		/* load from QFile */
		int loadFrom(QFile *, bool = false);

		/* proper name */
		QString getRobotName();

	private:
		/* process a core configuration element */
		void processElement(QDomElement, bool);

		/* process data */
		void processSensors(QDomElement);
        void processJoints(QDomElement);
        void processProcessedData(QDomElement);
		void processDiagnostics(QDomElement);
		void processCommands(QDomElement);
		void processControls(QDomElement);

		void addCamera(QDomElement);
		void addLaser(QDomElement);
		void addGPS(QDomElement);
		void addCompass(QDomElement);
		void addIMU(QDomElement);
		void addRange(QDomElement);
        void addJoint(QDomElement);
        void addImage(QDomElement);
        void addDisparityImage(QDomElement);
        void addMap(QDomElement);
        void addOdometry(QDomElement);
		void addTemperature(QDomElement);
		void addVoltage(QDomElement);
		void addBatteryLevel(QDomElement);
		void addCommandCustom(QDomElement);
		void addDrive(QDomElement);
};

#endif // CONTROL_PANEL_ROBOT_CONFIG_H
