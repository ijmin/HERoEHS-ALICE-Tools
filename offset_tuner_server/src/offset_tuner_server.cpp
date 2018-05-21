/*
 * offset_tuner_server.cpp
 *
 *  Created on: Mar 7, 2018
 *      Author: robotemperor
 */
#include "offset_tuner_server/offset_tuner_server.h"

#define OFFSET_ROSPARAM_KEY "offset"
#define OFFSET_INIT_POS_ROSPARAM_KEY "init_pose_for_offset_tuner"

using namespace offset;
using namespace alice;

OffsetTunerServer::OffsetTunerServer()
: controller(0),
  init_file(""),
  robot_file(""),
  offset_file("")
{
}

OffsetTunerServer::~OffsetTunerServer()
{
}

void OffsetTunerServer::setCtrlModule(std::string module)
{
	robotis_controller_msgs::JointCtrlModule control_msg;

	std::map<std::string, robotis_framework::DynamixelState *>::iterator joint_iter;
	ros::NodeHandle nh;
	ros::Publisher set_ctrl_module_pub = nh.advertise<robotis_controller_msgs::JointCtrlModule>("/robotis/set_ctrl_module", 1);

	BaseModule* base_module = BaseModule::getInstance();

	for (joint_iter = base_module->result_.begin(); joint_iter != base_module->result_.end(); ++joint_iter)
	{
		control_msg.joint_name.push_back(joint_iter->first);
		control_msg.module_name.push_back(module);
	}

	set_ctrl_module_pub.publish(control_msg);
}


bool OffsetTunerServer::initialize()
{
	controller = robotis_framework::RobotisController::getInstance();

	ros::NodeHandle n;

	offset_file = n.param<std::string>("offset_file_path","");
	robot_file = n.param<std::string>("robot_file_path","");
	init_file = n.param<std::string>("init_file_path","");

	/* gazebo simulation */
	controller->gazebo_mode_ = n.param<bool>("gazebo", false);
	if (controller->gazebo_mode_ == true)
	{
		std::string robot_name = n.param<std::string>("gazebo_robot_name", "");
		if (robot_name != "")
			controller->gazebo_robot_name_ = robot_name;

		ROS_WARN("GAZEBO_MODE!!!!!!!!!!!!!!!");
	}
	//base_module::BaseModule::getInstance()->gazebo_check = controller->gazebo_mode_;

	if ((offset_file == "") || (robot_file == ""))
	{
		ROS_ERROR("Failed to get file path");
		return -1;
	}
	//
	//Controller Initialize with robot file info
	if (controller->initialize(robot_file, init_file) == false)
	{
		ROS_ERROR("ROBOTIS Controller Initialize Fail!");
		return -1;
	}

	//controller_->LoadOffset(offset_file_);
	controller->addMotionModule((robotis_framework::MotionModule*) BaseModule::getInstance());

	//Initialize RobotOffsetData
	for (std::map<std::string, robotis_framework::Dynamixel *>::iterator robot_it = controller->robot_->dxls_.begin();
			robot_it != controller->robot_->dxls_.end(); robot_it++)
	{
		std::string joint_name = robot_it->first;
		robot_offset_data[joint_name] = new JointOffsetData();
		robot_torque_enable_data[joint_name] = true;
	}

	//Load Offset.yaml
	YAML::Node offset_yaml_node = YAML::LoadFile(offset_file.c_str());

	//Get Offset Data and Init_Pose for Offset Tuning
	YAML::Node offset_data_node = offset_yaml_node[OFFSET_ROSPARAM_KEY];
	YAML::Node offset_init_pose_node = offset_yaml_node[OFFSET_INIT_POS_ROSPARAM_KEY];


	//Initialize Offset Data in RobotOffsetData
	for (YAML::const_iterator node_it = offset_data_node.begin(); node_it != offset_data_node.end(); node_it++)
	{
		std::string joint_name = node_it->first.as<std::string>();
		double offset = node_it->second.as<double>();

		std::map<std::string, JointOffsetData*>::iterator robot_offset_data_it = robot_offset_data.find(joint_name);
		if (robot_offset_data_it != robot_offset_data.end())
			robot_offset_data[joint_name]->joint_init_offset_rad_ = offset;
		robot_offset_data[joint_name]->joint_offset_rad_ = offset;

	}

	//Initialize Init Pose for Offset Tuning in RobotOffsetData
	for (YAML::const_iterator node_it = offset_init_pose_node.begin(); node_it != offset_init_pose_node.end(); node_it++)
	{
		std::string joint_name = node_it->first.as<std::string>();
		double offset_init_pose = node_it->second.as<double>();

		std::map<std::string, JointOffsetData*>::iterator robot_offset_data_it = robot_offset_data.find(joint_name);
		if (robot_offset_data_it != robot_offset_data.end())
			robot_offset_data[joint_name]->joint_init_pos_rad_ = offset_init_pose;
	}

	int i = 0;
	for (std::map<std::string, JointOffsetData*>::iterator it = robot_offset_data.begin(); it != robot_offset_data.end(); it++)
	{
		std::string       joint_name = it->first;
		JointOffsetData  *joint_data = it->second;

		/*ROS_INFO_STREAM(i << " | " << joint_name << " : " << joint_data->joint_init_pos_rad_ << ", "
				<< joint_data->joint_offset_rad_);*/
		i++;
	}
	//Add here Communication
	command_state_sub = n.subscribe("/heroehs/command_state", 10, &OffsetTunerServer::CommandStateMsgsCallBack, this);
	joint_offset_state_sub =  n.subscribe("/heroehs/joint_offset_state", 10, &OffsetTunerServer::JointOffsetStateMsgsCallBack, this);
	moving_state_pub = n.advertise<std_msgs::Bool>("/heroehs/moving_state", 10);

	joint_torque_on_off_ser = n.advertiseService("/heroehs/joint_torque_on_off", &OffsetTunerServer::JointTorqueOnOffCallBack, this);
	joint_torque_on_off_array_ser = n.advertiseService("/heroehs/joint_torque_on_off_array", &OffsetTunerServer::JointTorqueOnOffArrayCallBack, this);
	present_joint_state_array_ser = n.advertiseService("/heroehs/present_joint_state_array", &OffsetTunerServer::PresentJointStateArrayCallBack, this);
	return true;
}
void OffsetTunerServer::CommandStateMsgsCallBack(const std_msgs::String::ConstPtr& msg)
{
	if (msg->data == "save")
	{
		YAML::Emitter out;
		std::map<std::string, double> offset;
		std::map<std::string, double> init_pose;
		for (std::map<std::string, JointOffsetData*>::iterator it = robot_offset_data.begin();
				it != robot_offset_data.end(); it++)
		{
			std::string       joint_name = it->first;
			JointOffsetData  *joint_data = it->second;

			offset[joint_name]    = joint_data->joint_offset_rad_; // edit one of the nodes
			init_pose[joint_name] = 0; // edit one of the nodes
		}

		out << YAML::BeginMap;
		out << YAML::Key << "offset" << YAML::Value << offset;
		out << YAML::Key << "init_pose_for_offset_tuner" << YAML::Value << init_pose;
		out << YAML::EndMap;
		std::ofstream fout(offset_file.c_str());
		fout << out.c_str(); // dump it back into the file

	}
	else if (msg->data == "init_offset_pose" || msg->data == "init_offset_pose_zero")
	{
		moveToInitialPose(msg->data);
	}
	else
		ROS_INFO_STREAM("Invalid Command : " << msg->data);

}
void OffsetTunerServer::moveToInitialPose(std::string command)
{
	moving_state_msg.data = 1;
	moving_state_pub.publish(moving_state_msg);

	BaseModule *base_module = BaseModule::getInstance();
	base_module->go_to_init_pose(command);
	controller->startTimer();

	while (base_module->is_moving_state)
	{
		if(!ros::ok())
		{
			controller->stopTimer();
			setCtrlModule("none");
		}

	}
	controller->stopTimer();

	moving_state_msg.data = 0;
	moving_state_pub.publish(moving_state_msg);

	while (controller->isTimerRunning())
		usleep(10 * 1000);

	if (controller->isTimerRunning())
	{
		ROS_INFO("Timer Running");
	}

	setCtrlModule("none");
}
void OffsetTunerServer::JointOffsetStateMsgsCallBack(const offset_tuner_msgs::JointOffsetState::ConstPtr& msg)
{
	if (controller->isTimerRunning())
	{
		ROS_ERROR("Timer is running now");
		return;
	}

	//goal position
	ROS_INFO_STREAM(msg->joint_name << " " << msg->joint_goal_value << " " );

	std::map<std::string, JointOffsetData*>::iterator it;
	it = robot_offset_data.find(msg->joint_name);
	if (it == robot_offset_data.end())
	{
		ROS_ERROR("Invalid Joint Name");
		return;
	}

	if (robot_torque_enable_data[msg->joint_name] == false)
	{
		ROS_ERROR_STREAM(msg->joint_name << "is turned off the torque");
		return;
	}

	double  goal_pose_rad   = controller->robot_->dxls_[msg->joint_name]->direction_*msg->joint_goal_value*DEGREE2RADIAN;
	int32_t goal_pose_value = controller->robot_->dxls_[msg->joint_name]->convertRadian2Value(goal_pose_rad);
	uint8_t dxl_error       = 0;
	int32_t comm_result     = COMM_SUCCESS;

	comm_result = controller->writeCtrlItem(msg->joint_name,
			controller->robot_->dxls_[msg->joint_name]->goal_position_item_->item_name_,
			goal_pose_value, &dxl_error);
	if (comm_result != COMM_SUCCESS)
	{
		ROS_ERROR("Failed to write goal position");
		return;
	}
	else
	{
		//robot_offset_data[msg->joint_name]->joint_init_pos_rad_  = msg->goal_value;
		//robot_offset_data[msg->joint_name]->joint_offset_rad_    = msg->joint_goal_value*DEGREE2RADIAN + robot_offset_data[msg->joint_name]->joint_init_offset_rad_;
		robot_offset_data[msg->joint_name]->joint_offset_rad_    = msg->joint_goal_value*DEGREE2RADIAN;
	}

	if (dxl_error != 0)
	{
		ROS_ERROR_STREAM("goal_pos_set : " << msg->joint_name << "  has error " << (int) dxl_error);
	}

	// robot_offset_data_[msg->joint_name]->p_gain_ = msg->p_gain;
	// robot_offset_data_[msg->joint_name]->i_gain_ = msg->i_gain;
	// robot_offset_data_[msg->joint_name]->d_gain_ = msg->d_gain;

}
bool OffsetTunerServer::JointTorqueOnOffCallBack(offset_tuner_msgs::JointTorqueOnOff::Request &req, offset_tuner_msgs::JointTorqueOnOff::Response &res)
{
	std::string joint_name = req.torque_command.joint_name;
	bool torque_enable = req.torque_command.joint_torque_on_off;

	int32_t comm_result = COMM_SUCCESS;
	uint8_t dxl_error = 0;
	uint8_t torque_enable_value = 0;

	if (torque_enable)
		torque_enable_value = 1;
	else
		torque_enable_value = 0;

	comm_result = controller->writeCtrlItem(joint_name,
			controller->robot_->dxls_[joint_name]->torque_enable_item_->item_name_,
			torque_enable_value, &dxl_error);
	if (comm_result != COMM_SUCCESS)
	{
		ROS_ERROR("Failed to write goal position");
	}
	else
	{
		robot_torque_enable_data[joint_name] = torque_enable;
	}

	if (dxl_error != 0)
	{
		ROS_ERROR_STREAM("goal_pos_set : " << joint_name << "  has error " << (int) dxl_error);
	}

	return true;
}
bool OffsetTunerServer::JointTorqueOnOffArrayCallBack(offset_tuner_msgs::JointTorqueOnOffArray::Request &req, offset_tuner_msgs::JointTorqueOnOffArray::Response &res)
{
	for (unsigned int i = 0; i < req.torque_command.size(); i++)
	{
		std::string joint_name = req.torque_command[i].joint_name;
		bool torque_enable = req.torque_command[i].joint_torque_on_off;
		ROS_INFO_STREAM(i << " " << joint_name << torque_enable);


		int32_t comm_result = COMM_SUCCESS;
		uint8_t dxl_error = 0;
		uint8_t torque_enable_value = 0;

		if (torque_enable)
			torque_enable_value = 1;
		else
			torque_enable_value = 0;

		comm_result = controller->writeCtrlItem(joint_name,
				controller->robot_->dxls_[joint_name]->torque_enable_item_->item_name_,
				torque_enable_value, &dxl_error);
		if (comm_result != COMM_SUCCESS)
		{
			ROS_ERROR("Failed to write goal position");
		}
		else
		{
			robot_torque_enable_data[joint_name] = torque_enable;
		}

		if (dxl_error != 0)
		{
			ROS_ERROR_STREAM("goal_pos_set : " << joint_name << "  has error " << (int) dxl_error);
		}

	}

	return true;
}
bool OffsetTunerServer::PresentJointStateArrayCallBack(offset_tuner_msgs::PresentJointStateArray::Request &req, offset_tuner_msgs::PresentJointStateArray::Response &res)
{
	ROS_INFO("GetPresentJointOffsetDataService Called");
	res.joint_data.clear();
	for (std::map<std::string, JointOffsetData*>::iterator it = robot_offset_data.begin();
			it != robot_offset_data.end(); it++)
	{
		std::string       joint_name = it->first;
		JointOffsetData  *joint_data = it->second;

		offset_tuner_msgs::PresentJointStateData joint_offset_pos;
		int32_t torque_enable = 0;
		int32_t present_pos_value = 0;
		uint8_t dxl_error         = 0;
		int     comm_result       = COMM_SUCCESS;

		comm_result = controller->readCtrlItem(joint_name,
				controller->robot_->dxls_[joint_name]->present_position_item_->item_name_,
				(uint32_t*) &present_pos_value,
				&dxl_error);
		if (comm_result != COMM_SUCCESS)
		{
			ROS_ERROR("Failed to read present pos");
			return false;
		}

		usleep(10*1000);

		comm_result = controller->readCtrlItem(joint_name,
				controller->robot_->dxls_[joint_name]->torque_enable_item_->item_name_,
				(uint32_t *) &torque_enable,
				&dxl_error);

		if (comm_result != COMM_SUCCESS)
		{
			ROS_ERROR("Failed to read present pos");
			return false;
		}
		else
		{
			if (dxl_error != 0)
			{
				ROS_ERROR_STREAM(joint_name << "  has error " << (int) dxl_error);
			}

			joint_offset_pos.joint_name    = joint_name;
			joint_offset_pos.torque_state  = torque_enable;
			joint_offset_pos.offset_data   = joint_data->joint_offset_rad_*RADIAN2DEGREE;
			joint_offset_pos.present_position_value = controller->robot_->dxls_[joint_name]->direction_*controller->robot_->dxls_[joint_name]->convertValue2Radian(present_pos_value)*RADIAN2DEGREE;

			res.joint_data.push_back(joint_offset_pos);
		}
	}
	return true;
}

































