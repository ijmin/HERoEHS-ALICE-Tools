/*
 * offset_tuner_server.h
 *
 *  Created on: Mar 7, 2018
 *      Author: robotemperor
 */

#ifndef HEROEHS_ROBOCUP_HEROEHS_TOOLS_OFFSET_TUNER_SERVER_INCLUDE_OFFSET_TUNER_SERVER_OFFSET_TUNER_SERVER_H_
#define HEROEHS_ROBOCUP_HEROEHS_TOOLS_OFFSET_TUNER_SERVER_INCLUDE_OFFSET_TUNER_SERVER_OFFSET_TUNER_SERVER_H_

#include <map>
#include <fstream>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include "robotis_controller/robotis_controller.h"
#include "offset_tuner_msgs/JointOffsetState.h"
#include "offset_tuner_msgs/JointTorqueOnOff.h"
#include "offset_tuner_msgs/JointTorqueOnOffArray.h"
#include "offset_tuner_msgs/PresentJointStateArray.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "alice_base_module/base_module.h"

namespace offset
{

class JointOffsetData
{
public:
	double joint_offset_rad_;
	double joint_init_offset_rad_;
	double joint_init_pos_rad_;
	int p_gain_;
	int i_gain_;
	int d_gain_;
	JointOffsetData()
	{
		joint_offset_rad_ = 0;
		joint_init_offset_rad_ = 0;
		joint_init_pos_rad_ = 0;
		p_gain_ = 32;
		i_gain_ = 0;
		d_gain_ = 0;

	}
	~JointOffsetData()
	{

	}
private:

};

class OffsetTunerServer: public robotis_framework::Singleton<OffsetTunerServer>
{
public:
	OffsetTunerServer();
	~OffsetTunerServer();

	bool initialize();
	//void moveToInitPose();


private:
	robotis_framework::RobotisController* controller;

	//ros communication
	ros::Subscriber command_state_sub;
	ros::Subscriber joint_offset_state_sub;
	ros::Publisher moving_state_pub;

	ros::ServiceServer joint_torque_on_off_ser;
	ros::ServiceServer joint_torque_on_off_array_ser;
	ros::ServiceServer present_joint_state_array_ser;

	//message
	std_msgs::Bool moving_state_msg;


	std::string init_file;
	std::string robot_file;
	std::string offset_file;

	std::map<std::string, bool>             robot_torque_enable_data;
	std::map<std::string, JointOffsetData*> robot_offset_data;

	void setCtrlModule(std::string module);
	void moveToInitialPose(std::string command);
	void CommandStateMsgsCallBack(const std_msgs::String::ConstPtr& msg);
	void JointOffsetStateMsgsCallBack(const offset_tuner_msgs::JointOffsetState::ConstPtr& msg);

	bool JointTorqueOnOffCallBack(offset_tuner_msgs::JointTorqueOnOff::Request &req,offset_tuner_msgs::JointTorqueOnOff::Response &res);
    bool JointTorqueOnOffArrayCallBack(offset_tuner_msgs::JointTorqueOnOffArray::Request &req,offset_tuner_msgs::JointTorqueOnOffArray::Response &res);
    bool PresentJointStateArrayCallBack(offset_tuner_msgs::PresentJointStateArray::Request &req,offset_tuner_msgs::PresentJointStateArray::Response &res);
};

}





#endif /* HEROEHS_ROBOCUP_HEROEHS_TOOLS_OFFSET_TUNER_SERVER_INCLUDE_OFFSET_TUNER_SERVER_OFFSET_TUNER_SERVER_H_ */
