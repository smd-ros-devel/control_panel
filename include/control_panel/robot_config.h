/* @todo Add license here */

/**
 * \file   robot_config.h
 * \date   Dec 5, 2011
 * \author Scott K Logan, Matt Richard
 * \brief  RobotConfig is the set of data that defines a robot's physical and sensory configuration.
 */
#ifndef CONTROL_PANEL_ROBOT_CONFIG_H
#define CONTROL_PANEL_ROBOT_CONFIG_H

#include <QObject>
#include <QFile>
#include <QDomDocument>
#include <QImage>


///////////////////////////// Sensors //////////////////////////////

struct RobotCamera
{
	public:
		/* linked list */
		struct RobotCamera *next;

		/* stored data */
		QString name;
		QString topicName;

		/* constructor */
		RobotCamera() : next(NULL), name("Unknown Camera"), topicName("unknown_camera")
		{
		}
};

struct RobotLaser
{
	public:
		/* linked list */
		struct RobotLaser *next;

		/* stored data */
		QString name;
		QString topicName;

		/* constructor */
		RobotLaser() : next(NULL), name("Unknown Laser"), topicName("unknown_laser")
		{
		}
};

struct RobotGPS
{
	public:
		/* linked list */
		struct RobotGPS *next;

		/* stored data */
		QString name;
		QString topicName;
		bool longitude;
		bool latitude;
		bool altitude;
		bool covariance;

		/* constructor */
		RobotGPS() : next(NULL), name("Unknown GPS"), topicName("unknown_gps"), longitude(false), latitude(false), altitude(false), covariance(false)
		{
		}
};

struct RobotCompass
{
	public:
		/* linked list */
		struct RobotCompass *next;

		/* stored data */
		QString name;
		QString topicName;

		/* constructor */
		RobotCompass() : next(NULL), name("Unknown Compass"), topicName("unknown_compass")
		{
		}
};

struct RobotIMU
{
	public:
		/* linked list */
		struct RobotIMU *next;

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
		RobotIMU() : next(NULL), name("Unknown IMU"), topicName("unknown_imu/data"), roll(false), pitch(false), yaw(false), angularVelocity(false), linearAcceleration(false), hideAttitude(false), hideHeading(false), hideLabels(false)
		{
		}
};

struct RobotRange
{
	public:
		/* linked list */
		struct RobotRange *next;

		/* stored data */
		QString name;
		QString topicName;

		/* constructor */
		RobotRange() : next(NULL), name("Unknown Range"), topicName("unknown_range")
		{
		}
};

struct RobotSensors
{
	public:
		/* sensor list */
		struct RobotCamera *cameras;
		struct RobotLaser *lasers;
		struct RobotGPS *gps;
		struct RobotCompass *compass;
		struct RobotIMU *imu;
		struct RobotRange *range;

		/* constructor */
		RobotSensors() : cameras(NULL), lasers(NULL), gps(NULL), compass(NULL), imu(NULL), range(NULL)
		{
		}

		/* destructor */
		~RobotSensors()
		{
			defaults();
		}

		/* set all to defaults */
		void defaults()
		{
			struct RobotCamera *tmp_camera;
			while(cameras != NULL)
			{
				tmp_camera = cameras;
				cameras = cameras->next;
				delete tmp_camera;
			}
			struct RobotLaser *tmp_laser;
			while(lasers != NULL)
			{
				tmp_laser = lasers;
				lasers = lasers->next;
				delete tmp_laser;
			}
			struct RobotGPS *tmp_gps;
			while(gps != NULL)
			{
				tmp_gps = gps;
				gps = gps->next;
				delete tmp_gps;
			}
			struct RobotCompass *tmp_compass;
			while(compass != NULL)
			{
				tmp_compass = compass;
				compass = compass->next;
				delete tmp_compass;
			}
			struct RobotIMU *tmp_imu;
			while(imu != NULL)
			{
				tmp_imu = imu;
				imu = imu->next;
				delete tmp_imu;
			}
			struct RobotRange *tmp_range;
			while(range != NULL)
			{
				tmp_range = range;
				range = range->next;
				delete tmp_range;
			}
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
        /* linked list */
        struct RobotJoint *next;

        /* stored data */
        QString name;
        QString displayName;

        /**
         * \brief Contstructor. Initializes linked list and data
         */
        RobotJoint()
            : next(NULL),
              name("unknown_joint"),
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
        struct RobotJoint *joints;

        /**
         * \brief Constructor. Initializes public members
         */
        RobotJoints()
            : topicName("joint_states"),
              position(false),
              velocity(false),
              effort(false),
              used(false),
              joints(NULL) { }

        /**
         * \brief Deconstructor. Calls the defaults function
         */
         ~RobotJoints() { defaults(); }

        /**
         * \brief Destroys linked list
         */
        void defaults()
        {
            topicName = "joint_states";
            position = false;
            velocity = false;
            effort = false;
            struct RobotJoint *tmp_joint;
            while(joints != NULL)
            {
                tmp_joint = joints;
                joints = joints->next;
                delete tmp_joint;
            }
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
        /* linked list */
        struct RobotDisparityImage *next;

        /* stored data */
        QString name;
        QString topicName;

        /**
         * \brief Constructor. Initilizes data.
         */
        RobotDisparityImage()
            : next(NULL),
              name("Unknown Disparity Image"),
              topicName("unknown_disparity_image") { }
};

/**
 * \struct RobotMap
 * \brief  @todo Fill this out
 */
struct RobotMap
{
    public:
        /* linked list */
        struct RobotMap *next;

        /* stored data */
        QString name;
        QString topicName;

        /**
         * \brief Constructor. Sets member values to default values.
         */
        RobotMap()
            : next(NULL),
              name("Unknown Map"),
              topicName("unknown_map") { }
};

/**
 * \struct RobotOdometry
 * \brief  @todo Fill this out
 */
struct RobotOdometry
{
    public:
        /* linked list */
        struct RobotOdometry *next;

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
            : next(NULL),
              name("Unknown Odometry"),
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
        struct RobotCamera *images;
        struct RobotDisparityImage *disparity_images;
        struct RobotMap *maps;
        struct RobotOdometry *odometry;

        /**
         * \brief Constructor
         */
        RobotProcessedData()
            : images(NULL),
              disparity_images(NULL),
              maps(NULL),
              odometry(NULL)
        {
        }

        /**
         * \brief Deconstructor. Calls default function.
         */
        ~RobotProcessedData() { defaults(); }

        /**
         * \brief Sets all struct members to their defaults
         */
        void defaults()
        {
            struct RobotCamera *tmp_image;
            while(images != NULL)
            {
                tmp_image = images;
                images = images->next;
                delete tmp_image;
            }
            struct RobotDisparityImage *tmp_disparity;
            while(disparity_images != NULL)
            {
                tmp_disparity = disparity_images;
                disparity_images = disparity_images->next;
                delete tmp_disparity;
            }
            struct RobotMap *tmp_map;
            while(maps != NULL)
            {
                tmp_map = maps;
                maps = maps->next;
                delete tmp_map;
            }
            struct RobotOdometry *tmp_odometry;
            while(odometry != NULL)
            {
                tmp_odometry = odometry;
                odometry = odometry->next;
                delete tmp_odometry;
            }
        }
};

///////////////////////// End Processed Data //////////////////////////


//////////////////////////// Diagnostics ////////////////////////////

struct RobotTemperature
{
	public:
		/* linked list */
		struct RobotTemperature *next;

		/* stored data */
		QString name;
		float minRange;
		float maxRange;
		QString units;

		/* constructor */
		RobotTemperature() : next(NULL), name("Unknown Temperature"), minRange(0), maxRange(0), units("Celcius")
		{
		}
};

struct RobotVoltage
{
	public:
		/* linked list */
		struct RobotVoltage *next;

		/* stored data */
		QString name;
		float minRange;
		float maxRange;
		float operatingVoltage;

		/* constructor */
		RobotVoltage() : next(NULL), name("Unknown Temperature"), minRange(0), maxRange(0), operatingVoltage(0)
		{
		}
};

struct RobotBatteryLevel
{
	public:
		/* linked list */
		struct RobotBatteryLevel *next;

		/* stored data */
		QString name;
		float max;

		/* constructor */
		RobotBatteryLevel() : next(NULL), name("Unknown Battery Level"), max(0)
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
		struct RobotTemperature *temperature;
		struct RobotVoltage *voltage;
		struct RobotBatteryLevel *batteryLevel;

		/* constructor */
		RobotDiagnostics() : topicName("diagnostics"), used(false), temperature(NULL), voltage(NULL), batteryLevel(NULL)
		{
		}

		/* destructor */
		~RobotDiagnostics()
		{
			defaults();
		}

		/* set all to defaults */
		void defaults()
		{
			topicName = "diagnostics";
			used = false;
			struct RobotTemperature *tmp_temperature;
			while(temperature != NULL)
			{
				tmp_temperature = temperature;
				temperature = temperature->next;
				delete tmp_temperature;
			}
			struct RobotVoltage *tmp_voltage;
			while(voltage != NULL)
			{
				tmp_voltage = voltage;
				voltage = voltage->next;
				delete tmp_voltage;
			}
			struct RobotBatteryLevel *tmp_batteryLevel;
			while(batteryLevel != NULL)
			{
				tmp_batteryLevel = batteryLevel;
				batteryLevel = batteryLevel->next;
				delete tmp_batteryLevel;
			}
		}
};

///////////////////////////// End Diagnostics /////////////////////////////


/////////////////////////// Commands //////////////////////////////////

struct RobotCommandCustom
{
	public:
		/* linked list */
		struct RobotCommandCustom *next;

		/* stored data */
		QString name;
		QString topicName;

		/* constructor */
		RobotCommandCustom() : next(NULL), name("Unknown Temperature"), topicName("unknown_custom_command")
		{
		}
};

struct RobotCommands
{
	public:
		/* stored data */
		bool used;

		/* commands list */
		struct RobotCommandCustom *custom;

		/* constructor */
		RobotCommands() : used(false), custom(NULL)
		{
		}

		/* destructor */
		~RobotCommands()
		{
			defaults();
		}

		/* set all to defaults */
		void defaults()
		{
			used = false;
			struct RobotCommandCustom *tmp_custom;
			while(custom != NULL)
			{
				tmp_custom = custom;
				custom = custom->next;
				delete tmp_custom;
			}
		}
};

////////////////////////// End Commands ///////////////////////////////


//////////////////////////// Controls ////////////////////////////////

struct RobotDrive
{
	public:
		/* linked list */
		struct RobotDrive *next;

		/* stored data */
		QString topicName;

		/* constructor */
		RobotDrive() : next(NULL), topicName("unknown_drive")
		{
		}
};

struct RobotControls
{
	public:
		/* stored data */
		bool used;

		/* controls list */
		struct RobotDrive *drive;

		/* constructor */
		RobotControls() : used(false), drive(NULL)
		{
		}

		/* destructor */
		~RobotControls()
		{
			defaults();
		}

		/* set all to defaults */
		void defaults()
		{
			used = false;
			struct RobotDrive *tmp_drive;
			while(drive != NULL)
			{
				tmp_drive = drive;
				drive = drive->next;
				delete tmp_drive;
			}
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
