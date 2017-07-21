/*
 * This file is part of the VSS-SDK project.
 *
 * This Source Code Form is subject to the terms of the GNU GENERAL PUBLIC LICENSE,
 * v. 3.0. If a copy of the GPL was not distributed with this
 * file, You can obtain one at http://www.gnu.org/licenses/gpl-3.0/.
 */

#include "interface.h"

Interface::Interface(){
}

//! Esse método deve ser chamado apenas uma vez
void Interface::createSendState(vss_sdk::Global_State *global_state){
  //! Global_State é recebido como ponteiro, assim facilitando o envio de novos estados
  this->global_state = global_state;
  nh_state.setCallbackQueue(&q_state);
  pub_state = nh_state.advertise<vss_sdk::Global_State>("global_state", 5);
}

//! Esse método deve ser chamado em um loop infinito controlado.
//! O envio tratado aqui é não bloqueante
void Interface::sendState(){
  pub_state = nh_state.publish(*global_state);
}

//! Esse método deve ser chamado apenas uma vez
void Interface::createReceiveState(vss_sdk::Global_State *global_state){
  this->global_state = global_state;
  nh_state.setCallbackQueue(&q_state);
  sub_state = nh_state.subscribe("global_state",callback, this);
}

void Interface::callbackReceiveState(const vss_sdk::Global_State::ConstPtr& msg) {
  this->global_state = msg;
}

//! Esse método deve ser chamado em um loop infinito controlado.
//! O recebimento tratado aqui é não bloqueante
void Interface::receiveState(){
  while (ros::ok()) q_state.callAvailable(ros::WallDuration());
}

//! Esse método deve ser chamado apenas uma vez
void Interface::createSendCommandsTeam1(vss_command::Global_Commands* global_command_yellow){
	//! Global_Commands é recebido como ponteiro, assim facilitando o envio de novos comandos
  this->global_command_yellow = global_command_yellow;
  nh_command_yellow.setCallbackQueue(&q_command_yellow);
  pub_command_yellow = nh_command_yellow.advertise<std_msgs::String>("global_command_yellow", 5);
}

//! Esse método deve ser chamado em um loop infinito controlado.
//! O envio tratado aqui é bloqueante
void Interface::sendCommandTeam1(){
	
}

//! Esse método deve ser chamado apenas uma vez
void Interface::createSendCommandsTeam2(vss_command::Global_Commands* global_commands){
	//! Global_Commands é recebido como ponteiro, assim facilitando o envio de novos comandos
	this->global_command_ = global_commands;
	this->addr_client_simulator_team2 = addr_client_simulator_team2;

	context_command_blue = new zmq::context_t(1);
	socket_command_blue = new zmq::socket_t(*context_command_blue, ZMQ_PAIR);

	std::cout << "Connecting Client Sender Team 2: " << addr_client_simulator_team2 << "(blue team)" << std::endl << std::endl;
	socket_command_blue->connect(addr_client_simulator_team2.c_str());
}

//! Esse método deve ser chamado em um loop infinito controlado.
//! O envio tratado aqui é bloqueante
void Interface::sendCommandTeam2(){
	//! Chamando esse método, o Global_Commands passado por ponteiro na construção do socket automaticamente é atualizado
	std::string msg_str;
	global_commands->SerializeToString(&msg_str);

	zmq::message_t request (msg_str.size());
	memcpy ((void *) request.data (), msg_str.c_str(), msg_str.size());
	//std::cout << "Sending State data ..." << std::endl;
	socket_command_blue->send(request);
	//printCommand();
}

//! Esse método deve ser chamado apenas uma vez
void Interface::createReceiveCommandsTeam1(vss_command::Global_Commands* global_commands){
	//! Global_Commands é recebido como ponteiro, assim facilitando o recebimento de novos comandos
	this->global_commands = global_commands;
	this->addr_server_simulator_team1 = addr_server_simulator_team1;

	context_command_yellow = new zmq::context_t(1);
	socket_command_yellow = new zmq::socket_t(*context_command_yellow, ZMQ_PAIR);

	std::cout << "Connecting Server Receiver Team 1: " << addr_server_simulator_team1 << std::endl;
	socket_command_yellow->bind(addr_server_simulator_team1.c_str());
}

//! Esse método deve ser chamado em um loop infinito controlado.
//! O recebimento tratado aqui é bloqueante
void Interface::receiveCommandTeam1(){
	//! Chamando esse método, o Global_Commands passado por ponteiro na construção do socket automaticamente é atualizado
	zmq::message_t request;
	socket_command_yellow->recv(&request);
	//std::cout << "Received" << std::endl;
	std::string msg_str(static_cast<char*>(request.data()), request.size());
	global_commands->ParseFromString(msg_str);
	//printCommand();
	//socket.close();
}

//! Esse método deve ser chamado apenas uma vez
void Interface::createReceiveCommandsTeam2(vss_command::Global_Commands* global_commands){
	//! Global_Commands é recebido como ponteiro, assim facilitando o envio de novos comandos
	this->global_commands = global_commands;
	this->addr_server_simulator_team2 = addr_server_simulator_team2;

	context_command_blue = new zmq::context_t(1);
	socket_command_blue = new zmq::socket_t(*context_command_blue, ZMQ_PAIR);

	std::cout << "Connecting Server Receiver Team 2: " << addr_server_simulator_team2 << std::endl;
	socket_command_blue->bind(addr_server_simulator_team2.c_str());
}

//! Esse método deve ser chamado em um loop infinito controlado.
//! O recebimento tratado aqui é bloqueante
void Interface::receiveCommandTeam2(){
	//! Chamando esse método, o Global_Commands passado por ponteiro na construção do socket automaticamente é atualizado
	zmq::message_t request;
	socket_command_blue->recv(&request);
	//std::cout << "Received" << std::endl;
	std::string msg_str(static_cast<char*>(request.data()), request.size());
	global_commands->ParseFromString(msg_str);
	//printCommand();
}

//! Esse método deve ser chamado apenas uma vez
void Interface::createSendDebugTeam1(vss_debug::Global_Debug* global_debug){
	//! Global_Debug é recebido como ponteiro, assim facilitando o envio de novas informações de debug
	this->global_debug = global_debug;
	this->addr_client_debug_team1 = addr_client_debug_team1;

	context_debug = new zmq::context_t(1);
	socket_debug = new zmq::socket_t(*context_debug, ZMQ_PAIR);

	std::cout << "Connecting Server Sender Debug Team 1: " << addr_client_debug_team1 << "(yellow team)" << std::endl << std::endl;
	socket_debug->connect(addr_client_debug_team1.c_str());
}

//! Esse método deve ser chamado em um loop infinito controlado.
//! O envio tratado aqui é bloqueante
void Interface::sendDebugTeam1(){
	//! Chamando esse método, o Global_Debug passado por ponteiro na construção do socket automaticamente é atualizado
	std::string msg_str;
	global_debug->SerializeToString(&msg_str);

	zmq::message_t request (msg_str.size());
	memcpy ((void *) request.data (), msg_str.c_str(), msg_str.size());
	//std::cout << "Sending State data ..." << std::endl;
	socket_debug->send(request);
	//printCommand();
}

//! Esse método deve ser chamado apenas uma vez
void Interface::createReceiveDebugTeam1(vss_debug::Global_Debug* global_debug){
	//! Global_Debug é recebido como ponteiro, assim facilitando o recebimento de novas informações de debug
	this->global_debug = global_debug;
	this->addr_server_debug_team1 = addr_server_debug_team1;

	context_debug = new zmq::context_t(1);
	socket_debug = new zmq::socket_t(*context_debug, ZMQ_PAIR);

	std::cout << "Connecting Client Receiver Debug Team 1: " << addr_server_debug_team1 << std::endl;
	socket_debug->bind(addr_server_debug_team1.c_str());
}

//! Esse método deve ser chamado em um loop infinito controlado.
//! O recebimento tratado aqui é bloqueante
void Interface::receiveDebugTeam1(){
	//! Chamando esse método, o Global_Debug passado por ponteiro na construção do socket automaticamente é atualizado
	zmq::message_t request;
	socket_debug->recv(&request);
	//std::cout << "Received" << std::endl;
	std::string msg_str(static_cast<char*>(request.data()), request.size());
	global_debug->ParseFromString(msg_str);
	//printCommand();
	//socket.close();
}

//! Esse método deve ser chamado apenas uma vez
void Interface::createSendDebugTeam2(vss_debug::Global_Debug* global_debug){
	//! Global_Debug é recebido como ponteiro, assim facilitando o envio de novas informações de debug
	this->global_debug = global_debug;
	this->addr_client_debug_team2 = addr_client_debug_team2;

	context_debug = new zmq::context_t(1);
	socket_debug = new zmq::socket_t(*context_debug, ZMQ_PAIR);

	std::cout << "Connecting Server Sender Debug Team 2: " << addr_client_debug_team2 << " (blue team)" << std::endl << std::endl;
	socket_debug->connect(addr_client_debug_team2.c_str());
}

//! Esse método deve ser chamado em um loop infinito controlado.
//! O envio tratado aqui é bloqueante
void Interface::sendDebugTeam2(){
	//! Chamando esse método, o Global_Debug passado por ponteiro na construção do socket automaticamente é atualizado
	std::string msg_str;
	global_debug->SerializeToString(&msg_str);

	zmq::message_t request (msg_str.size());
	memcpy ((void *) request.data (), msg_str.c_str(), msg_str.size());
	//std::cout << "Sending State data ..." << std::endl;
	socket_debug->send(request);
	//printCommand();
}

//! Esse método deve ser chamado apenas uma vez
void Interface::createReceiveDebugTeam2(vss_debug::Global_Debug* global_debug){
	//! Global_Debug é recebido como ponteiro, assim facilitando o recebimento de novas informações de debug
	this->global_debug = global_debug;
	this->addr_server_debug_team2 = addr_server_debug_team2;

	context_debug = new zmq::context_t(1);
	socket_debug = new zmq::socket_t(*context_debug, ZMQ_PAIR);

	std::cout << "Connecting Client Receiver Debug Team 2: " << addr_server_debug_team2 << std::endl;
	socket_debug->bind(addr_server_debug_team2.c_str());
}

//! Esse método deve ser chamado em um loop infinito controlado.
//! O recebimento tratado aqui é bloqueante
void Interface::receiveDebugTeam2(){
	//! Chamando esse método, o Global_Debug passado por ponteiro na construção do socket automaticamente é atualizado
	zmq::message_t request;
	socket_debug->recv(&request);
	//std::cout << "Received" << std::endl;
	std::string msg_str(static_cast<char*>(request.data()), request.size());
	global_debug->ParseFromString(msg_str);
	//printCommand();
	//socket.close();
}

void Interface::printState(){
	std::string text_str;
    google::protobuf::TextFormat::PrintToString(*global_state, &text_str);
    std::cout << text_str << std::endl;
}

void Interface::printCommand(){
	std::string text_str;
    google::protobuf::TextFormat::PrintToString(*global_commands, &text_str);
    std::cout << text_str << std::endl;
}

void Interface::printDebug(){
	std::string text_str;
    google::protobuf::TextFormat::PrintToString(*global_debug, &text_str);
    std::cout << text_str << std::endl;
}
