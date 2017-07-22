/*
 * This file is part of the VSS-SDK project.
 *
 * This Source Code Form is subject to the terms of the GNU GENERAL PUBLIC LICENSE,
 * v. 3.0. If a copy of the GPL was not distributed with this
 * file, You can obtain one at http://www.gnu.org/licenses/gpl-3.0/.
 */

#include "interface.h"

//! Esse método deve ser chamado apenas uma vez
template <class msg_Type>
void Interface<msg_Type>::createSend(msg_Type *message, std::string nodeName){
  //! Global_State é recebido como ponteiro, assim facilitando o envio de novos estados
  msg = message;
  nh.setCallbackQueue(&q);
  pub = nh.advertise<msg_Type>(nodeName, 5);
}

//! Esse método deve ser chamado em um loop infinito controlado.
//! O envio tratado aqui é não bloqueante
template <class msg_Type>
void Interface<msg_Type>::send(){
  pub.publish(*msg);
}

//! Esse método deve ser chamado apenas uma vez
template <class msg_Type>
void Interface<msg_Type>::createReceive(msg_Type *message, std::string nodeName){
  msg = message;
  nh.setCallbackQueue(&q);
  sub = nh.subscribe(nodeName, 5, &Interface3::callback, this);
}

template <class msg_Type>
void Interface<msg_Type>::callback(const msg_Type message) {
  *msg = message;
}

//! Esse método deve ser chamado em um loop infinito controlado.
//! O recebimento tratado aqui é não bloqueante
template <class msg_Type>
void Interface<msg_Type>::receive(){
  while (ros::ok()) q_state.callAvailable(ros::WallDuration());
}

//TODO verificar metodos print*

void Interface<msg_Type>::printState(){
	std::string text_str;
    google::protobuf::TextFormat::PrintToString(*global_state, &text_str);
    std::cout << text_str << std::endl;
}

void Interface<msg_Type>::printCommand(){
	std::string text_str;
    google::protobuf::TextFormat::PrintToString(*global_commands, &text_str);
    std::cout << text_str << std::endl;
}

void Interface<msg_Type>::printDebug(){
	std::string text_str;
    google::protobuf::TextFormat::PrintToString(*global_debug, &text_str);
    std::cout << text_str << std::endl;
}
