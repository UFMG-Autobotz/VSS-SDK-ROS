/*
* This file is part of the VSS-SDK project.
*
* This Source Code Form is subject to the terms of the GNU GENERAL PUBLIC LICENSE,
* v. 3.0. If a copy of the GPL was not distributed with this
* file, You can obtain one at http://www.gnu.org/licenses/gpl-3.0/.
*/

#ifndef _INTERFACE_H_
#define _INTERFACE_H_

#include "ros/ros.h"
#include <ros/callback_queue.h>

#include <sstream>
#include <iostream>
#include <string>
#include "unistd.h"

#include "vss_sdk_ros/global_state.h"
#include "vss_sdk_ros/s_ball_state.h"
#include "vss_sdk_ros/s_pose.h"
#include "vss_sdk_ros/s_rgb.h"
#include "vss_sdk_ros/s_robot_state.h"

#include "vss_sdk_ros/global_commands.h"
#include "vss_sdk_ros/c_robot_command.h"

#include "vss_sdk_ros/global_debug.h"
#include "vss_sdk_ros/d_path.h"
#include "vss_sdk_ros/d_pose.h"

using namespace std;

//! Essa classe define as interfaces utilizadas em todos os projetos do VSS-SDK
// cada objeto deve apenas enviar ou receber um tipo de mensagem
template <class msg_Type>
class Interface{
protected:
  ros::NodeHandle nh;
  ros::CallbackQueue q;  // para evitar conflitos, cada mensagem tem sua propria fila
  ros::Publisher pub;
  ros::Subscriber sub;

  msg_Type *msg;

public:
  void createSend(msg_Type *message, std::string nodeName);
  void send();
  void createReceive(msg_Type *message, std::string nodeName);
  void callback(const msg_Type message);
  void receive();
};

#endif // _INTERFACE_H_
