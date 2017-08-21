/*
 * This file is part of the VSS-SampleStrategy project.
 *
 * This Source Code Form is subject to the terms of the GNU GENERAL PUBLIC LICENSE,
 * v. 3.0. If a copy of the GPL was not distributed with this
 * file, You can obtain one at http://www.gnu.org/licenses/gpl-3.0/.
 */

#include "sample.h"

// para evitar erros de ligaçao:
//https://www.codeproject.com/Articles/48575/How-to-define-a-template-class-in-a-h-file-and-imp
#include "../Interface/interface.cpp"

Sample::Sample(){
}

void Sample::init_sample(string main_color, bool is_debug, bool real_environment, string ip_receive_state, string ip_send_debug, string ip_send_command, string name){
	flag_init = 0;
	this->main_color = main_color;
	this->is_debug = is_debug;
	this->real_environment = real_environment;
	this->name = name;


  interface.createReceive(&global_state, "state");

	if(main_color == "yellow") {
		interface_send.createSend(&global_commands, "commandsYellow");

		if(is_debug) {
			interface_debug.createSend(&global_debug, "debugYellow");
		}

	}else{
		interface_send.createSend(&global_commands, "commandsBlue");

		if(is_debug) {
			interface_debug.createSend(&global_debug, "debugBlue");
		}
	}
}

void Sample::receive_state(){
	interface.receive();
	state = common::global_state2State(global_state, main_color);
	situation = global_state.situation;
}

void Sample::send_commands() {
	global_commands.situation = NONE;

	if(flag_init == 0) {
		global_commands.name = name;
		flag_init = 1;
	}

	global_commands.is_team_yellow = main_color == "yellow";

	for(int i = 0; i < 3; i++) {
    vss_sdk_ros::c_robot_command robot;
		robot.id = i;
		robot.left_vel = commands[i].left;
		robot.right_vel = commands[i].right;
    global_commands.robot_commands[i] = robot;
	}

	interface_send.send();
}

void Sample::send_debug() {

	// Add step pose, if exists
	for (int i = 0; i < 3; i++) {
    vss_sdk_ros::d_pose steps;
		steps.id = i;
		steps.x = debug.robots_step_pose[i].x;
		steps.y = debug.robots_step_pose[i].y;
		steps.yaw = debug.robots_step_pose[i].z;
    global_debug.step_poses[i] = steps;
	}

	// Add final pose, if exists
	for (int i = 0; i < 3; i++) {
    vss_sdk_ros::d_pose finals;
		finals.id = i;
		finals.x = debug.robots_final_pose[i].x;
		finals.y = debug.robots_final_pose[i].y;
		finals.yaw = debug.robots_final_pose[i].z;
    global_debug.final_poses[i] = finals;
	}

	for(int i = 0; i < 3; i++) {
    vss_sdk_ros::d_path paths;
		paths.id = i;
		for(int j = 0; j < debug.robots_path[i].poses.size(); j++) {
			vss_sdk_ros::d_pose poses;
			poses.id = i;
			poses.x = debug.robots_path[i].poses.at(j).x;
			poses.y = debug.robots_path[i].poses.at(j).y;
			poses.yaw = debug.robots_path[i].poses.at(j).z;
      paths.poses.push_back(poses);
		}
    global_debug.paths[i] = paths;
	}

	interface_debug.send();
}
