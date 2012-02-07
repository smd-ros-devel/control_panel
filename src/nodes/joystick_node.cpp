/*****************************************************************************
** joystick_node.cpp
** 
** Author:      Scott K Logan, Matt Richard
** Date:        October 2011
** Description: SRS Basestation interface to joystick_driver stack.
*****************************************************************************/

#include "control_panel/nodes/joystick_node.h"

/******************************************************************************
 * Function:    JoystickNode
 * Author:      Scott Logan
 * Parameters:  None
 * Returns:     None
 * Description: Constructor.
 *****************************************************************************/
JoystickNode::JoystickNode() : nh(NULL)
{
	cmd_vel = new geometry_msgs::Twist;

	useJoy = false;
	joy_topic_name = Globals::DEFAULT_JOYSTICK_TOPIC;
	maxScale = 100;

	cmd_vel->linear.x = 0.0;
	cmd_vel->linear.y = 0.0;
	cmd_vel->linear.z = 0.0;
	cmd_vel->angular.x = 0.0;
	cmd_vel->angular.y = 0.0;
	cmd_vel->angular.z = 0.0;

}

/******************************************************************************
 * Function:    ~JoystickNode
 * Author:      Scott Logan
 * Parameters:  None
 * Returns:     None
 * Description: Deconstructor.
 *****************************************************************************/
JoystickNode::~JoystickNode()
{
	JoystickNode::disableJoystick();
	delete cmd_vel;
    delete nh;
}

/******************************************************************************
 * Function:    isEnable
 * Author:      Scott Logan
 * Parameters:  None
 * Returns:     bool -
 * Description:
 *****************************************************************************/
bool JoystickNode::isEnabled()
{
	return useJoy;
}

/******************************************************************************
 * Function:    enableJoystick
 * Author:      Scott Logan
 * Parameters:  None
 * Returns:     void
 * Description:
 *****************************************************************************/
void JoystickNode::enableJoystick()
{
	if(!useJoy)
		useJoy = true;

	if(!isRunning())
		start();
}

/******************************************************************************
 * Function:    run
 * Author:      Scott Logan
 * Parameters:  None
 * Returns:     void
 * Description:
 *****************************************************************************/
void JoystickNode::run()
{
    if(nh == NULL)
        nh = new ros::NodeHandle;

	joy_sub = nh->subscribe(joy_topic_name, 1, &JoystickNode::joyCallback, this);
	ros::spin();
}

/******************************************************************************
 * Function:    joyCallback
 * Author:      Scott Logan
 * Parameters:  const sensor_msgs::JoyConstPtr &msg -
 * Returns:     void
 * Description:
 *****************************************************************************/
void JoystickNode::joyCallback(const sensor_msgs::JoyConstPtr &msg)
{
	if (useJoy)
	{
		unsigned int i;

		static QVector<double> axes(msg->axes.size(), 0);
		static QVector<bool> buttons(msg->buttons.size(), false);

		if((unsigned)axes.size() != msg->axes.size())
			axes.fill(msg->axes.size(), 0);
		if((unsigned)buttons.size() != msg->buttons.size())
			buttons.fill(msg->buttons.size(), false);

		for(i = 0; i < msg->axes.size(); i++)
		{
			if(msg->axes[i] != axes[i])
			{
				axes[i] = msg->axes[i];
//				std::cout << "Axis " << i << ": " << msg->axes[i] << std::endl;
				emit axis_event(i, msg->axes[i]);
			}
		}

		for(i = 0; i < msg->buttons.size(); i++)
		{
			if(msg->buttons[i] != buttons[i])
			{
				buttons[i] = msg->buttons[i];
//				std::cout << "Buttons " << i << ": " << buttons[i] << std::endl;
				emit button_event(i, msg->buttons[i]);
			}
		}
//		cmd_vel->linear.y = maxScale * msg->axes[0] / 100;	//turn
//		cmd_vel->linear.x = maxScale * msg->axes[1] / 100;	//fwd-rev
//		cmd_vel->angular.z = maxScale * msg->axes[2] / 100;	//strafe
//		cmd_vel->angular.y = maxScale * msg->axes[3] / 100;	//pitch
//
//		emit velCmdReceived(msg->axes[1], msg->axes[0], 0.0, 
//			0.0, msg->axes[3], msg->axes[2]);
		
	}
}

/******************************************************************************
 * Function:    stopJoystick
 * Author:      Scott Logan
 * Parameters:  None
 * Returns:     void
 * Description:
 *****************************************************************************/
void JoystickNode::stopJoystick()
{
	disableJoystick();
	if(isRunning())
	{
		nh->shutdown();
		terminate();
	}
}

/******************************************************************************
 * Function:    disableJoystick
 * Author:      Scott Logan
 * Parameters:  None
 * Returns:     void
 * Description:
 *****************************************************************************/
void JoystickNode::disableJoystick()
{
	useJoy = false;
}
