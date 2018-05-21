/*
 * main.cpp
 *
 *  Created on: Mar 7, 2018
 *      Author: robotemperor
 */
#include <ros/ros.h>

#include "offset_tuner_server/offset_tuner_server.h"

using namespace offset;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "offset_tuner_server_node");

	OffsetTunerServer* server = OffsetTunerServer::getInstance();

	server->initialize();

	while ( ros::ok() )
	{

		ros::spinOnce();

	}

	return 0;
}



