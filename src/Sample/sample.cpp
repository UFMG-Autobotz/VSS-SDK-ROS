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
		interface_send.createSendCommandsTeam2(&global_commands, "commandsBlue");

		if(is_debug) {
			interface_debug.createSendDebugTeam2(&global_debug, "debugBlue");
		}
	}
}

void Sample::receive_state(){
	interface_receive.receiveState();
	state = common::Global_State2State(global_state, main_color);
	situation = global_state.situation();
}

void Sample::send_commands(){
	global_commands = vss_command::Global_Commands();
	global_commands.set_situation(NONE);

	if(flag_init == 0) {
		global_commands.set_name(name);
		flag_init = 1;
	}

	if(main_color == "yellow") {
		global_commands.set_is_team_yellow(true);
	}else{
		global_commands.set_is_team_yellow(false);
	}

	for(int i = 0; i < 3; i++) {
		vss_command::Robot_Command *robot = global_commands.add_robot_commands();
		robot->set_id(i);
		robot->set_left_vel(commands[i].left);
		robot->set_right_vel(commands[i].right);
	}

	if(main_color == "yellow") {
		interface_send.sendCommandTeam1();
	}else{
		interface_send.sendCommandTeam2();
	}
}

void Sample::send_debug(){
	global_debug = vss_debug::Global_Debug();

	// Add step pose, if exists
	for(int i = 0; i < 3; i++) {
		vss_debug::Pose *steps = global_debug.add_step_poses();
		steps->set_id(i);
		steps->set_x(debug.robots_step_pose[i].x);
		steps->set_y(debug.robots_step_pose[i].y);
		steps->set_yaw(debug.robots_step_pose[i].z);
	}

	// Add final pose, if exists
	for(int i = 0; i < 3; i++) {
		vss_debug::Pose *finals = global_debug.add_final_poses();
		finals->set_id(i);
		finals->set_x(debug.robots_final_pose[i].x);
		finals->set_y(debug.robots_final_pose[i].y);
		finals->set_yaw(debug.robots_final_pose[i].z);
	}

	for(int i = 0; i < 3; i++) {
		vss_debug::Path *paths = global_debug.add_paths();
		paths->set_id(i);
		for(int j = 0; j < debug.robots_path[i].poses.size(); j++) {
			vss_debug::Pose *poses = paths->add_poses();
			poses->set_id(i);
			poses->set_x(debug.robots_path[i].poses.at(j).x);
			poses->set_y(debug.robots_path[i].poses.at(j).y);
			poses->set_yaw(debug.robots_path[i].poses.at(j).z);
		}
	}

	if(main_color == "yellow") {
		interface_debug.sendDebugTeam1();
	}else{
		interface_debug.sendDebugTeam2();
	}
}
