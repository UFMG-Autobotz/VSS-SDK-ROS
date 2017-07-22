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

//REVIEW confirir nomes das mensagens
#include "vss_sdk/Global_State.h"
#include "vss_sdk/Global_Commands.h"
#include "vss_sdk/Global_Debug.h"

using namespace std;

//! Essa classe define as interfaces utilizadas em todos os projetos do VSS-SDK
template <class msg_Type>
class Interface{
protected:
  ros::NodeHandle nh;
  ros::CallbackQueue q;
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
