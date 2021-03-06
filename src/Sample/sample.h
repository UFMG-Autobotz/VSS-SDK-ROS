/*
 * This file is part of the VSS-SampleStrategy project.
 *
 * This Source Code Form is subject to the terms of the GNU GENERAL PUBLIC LICENSE,
 * v. 3.0. If a copy of the GPL was not distributed with this
 * file, You can obtain one at http://www.gnu.org/licenses/gpl-3.0/.
 */

#ifndef _SAMPLE_H_
#define _SAMPLE_H

#include "common.h"
#include "../Interface/interface.h"
#include "vector"

using namespace std;
using namespace common;

class Sample {
private:
Interface<vss_sdk_ros::global_state> interface;
Interface<vss_sdk_ros::global_commands> interface_send;
Interface<vss_sdk_ros::global_debug> interface_debug;

vss_sdk_ros::global_state global_state;
vss_sdk_ros::global_commands global_commands;
vss_sdk_ros::global_debug global_debug;

protected:
string main_color;
bool is_debug;
bool real_environment;
int situation;
string name;
int flag_init;

common::State state;
common::Command commands[3];
common::Debug debug;

public:
Sample();

void init_sample(string main_color, bool is_debug, bool real_environment, string ip_receive_state, string ip_send_debug, string ip_send_command, string name);
void receive_state();
void send_commands();
void send_debug();
};

#endif // _SAMPLE_H_
