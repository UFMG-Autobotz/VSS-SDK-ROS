/*
 * This file is part of the VSS-SDK project.
 *
 * This Source Code Form is subject to the terms of the GNU GENERAL PUBLIC LICENSE,
 * v. 3.0. If a copy of the GPL was not distributed with this
 * file, You can obtain one at http://www.gnu.org/licenses/gpl-3.0/.
 */

#ifndef _INTERFACE_H_
#define _INTERFACE_H_

#include <sstream>
#include <iostream>
#include <string>
#include "unistd.h"

using namespace std;

//! Essa classe define as interfaces utilizadas em todos os projetos do VSS-SDK, em outras palavras, todos os sockets abertos em todos os projetos estão aqui.
class Interface{
protected:
	//! Contexto do socket de envio de estados pelo VSS-Vision. E contexto do socket de recebimento de estados pelos VSS-SampleStrategys
	zmq::context_t *context;
	//! Socket de envio de estados pelo VSS-Vision. E socket de recebimento de estados pelos VSS-SampleStrategys
    zmq::socket_t *socket;

	//! (Time Amarelo) Contexto do socket de envio de comandos por um VSS-SampleStrategy ou VSS-Joystick. E contexto do socket de recebimento de comandos pelo VSS-Simulator
	zmq::context_t *context_command_yellow;
	//! (Time Amarelo) Socket de envio de comandos por um VSS-SampleStrategy ou VSS-Joystick. E socket de recebimento de comandos pelo VSS-Simulator
    zmq::socket_t *socket_command_yellow;

	//! (Time Azul) Contexto do socket de envio de comandos por um VSS-SampleStrategy ou VSS-Joystick. E contexto do socket de recebimento de comandos pelo VSS-Simulator
	zmq::context_t *context_command_blue;
	//! (Time Azul) Socket de envio de comandos por um VSS-SampleStrategy ou VSS-Joystick. E socket de recebimento de comandos pelo VSS-Simulator
    zmq::socket_t *socket_command_blue;

	//! Contexto do socket de envio de informações de debug por um VSS-SampleStrategy. E contexto do socket de recebimento de informações de debug pelo VSS-Viewer
	zmq::context_t *context_debug;
	//! Socket de envio de informações de debug por um VSS-SampleStrategy. E socket de recebimento de informações de debug pelo VSS-Viewer
	zmq::socket_t *socket_debug;

	//! Pacote de estados (Utilizado pelo VSS-Simulator e VSS-Vision, para enviar informações do campo) (Utilizado também pelo VSS-Viewer, para desenhar a estado do jogo e pelo VSS-SampleStrategy para construir-se uma estratégia)
	vss_sdk::Global_State *global_state;

	//! Pacote de comandos (Utilizado por VSS-SampleStrategys e VSS-Joysticks, para enviar comandos para robôs virtuais) (Utilizado também pelo VSS-Simulator, para receber os comandos de cada robô)
	vss_sdk::Global_Commands *global_commands;

	//! Pacote de debug visual (Utilizado por VSS-SampleStrategys, para enviar informações da estratégia para o VSS-Viewer desenhar)
	vss_sdk::Global_Debug *global_debug;

public:
	//! Construtor DEFAULT
	Interface();

	//! Método responsável por criar o socket de envio de estados no VSS-Vision e VSS-Simulator
	void createSendState(vss_state::Global_State*, string addr_server_multicast = "tcp://*:5555");
	//! Método respontável por enviar um novo estado pelo VSS-Vision e VSS-Simulator
	void sendState();
	//! Método responsável por criar o socket de recebimento de estados em VSS-SampleStrategys
	void createReceiveState(vss_state::Global_State*, string addr_client_multicast = "tcp://localhost:5555");
	//! Método responsável por receber um novo estado em VSS-SampleStrategys
	void receiveState();

	//! Método responsável por criar o socket de envio de comandos em VSS-SampleStrategys ou VSS-Joysticks (Time Amarelo)
	void createSendCommandsTeam1(vss_command::Global_Commands*, string addr_client_simulator_team1 = "tcp://localhost:5556");
	//! Método responsável por enviar um novo comando por VSS-SampleStrategys ou VSS-Joysticks (Time Amarelo)
	void sendCommandTeam1();
	//! Método responsável por criar o socket de recebimento de comandos no VSS-Simulator (Time Amarelo)
	void createReceiveCommandsTeam1(vss_command::Global_Commands*, string addr_server_simulator_team1 = "tcp://*:5556");
	//! Método repsonsável por receber um novo comando (Time Amarelo)
	void receiveCommandTeam1();

	//! Método responsável por criar o socket de envio de comandos em VSS-SampleStrategys ou VSS-Joysticks (Time Azul)
	void createSendCommandsTeam2(vss_command::Global_Commands*, string addr_client_simulator_team2 = "tcp://localhost:5557");
	//! Método responsável por enviar um novo comando por VSS-SampleStrategys ou VSS-Joysticks (Time Azul)
	void sendCommandTeam2();
	//! Método responsável por criar o socket de recebimento de comandos no VSS-Simulator (Time Azul)
	void createReceiveCommandsTeam2(vss_command::Global_Commands*, string addr_server_simulator_team2 = "tcp://*:5557");
	//! Método repsonsável por receber um novo comando (Time Azul)
	void receiveCommandTeam2();

	//! Método responsável por criar o socket de envio de informações de debug em VSS-SampleStrategys ou VSS-Joysticks (Time Amarelo)
	void createSendDebugTeam1(vss_debug::Global_Debug*, string addr_client_debug_team1 = "tcp://localhost:5558");
	//! Método responsável por enviar uma nova informação de debug por VSS-SampleStrategys ou VSS-Joysticks (Time Amarelo)
	void sendDebugTeam1();
	//! Método responsável por criar o socket de recebimento de informações de debug no VSS-Simulator (Time Amarelo)
	void createReceiveDebugTeam1(vss_debug::Global_Debug*, string addr_server_debug_team1 = "tcp://*:5558");
	//! Método repsonsável por receber uma nova informação de debug (Time Amarelo)
	void receiveDebugTeam1();

	//! Método responsável por criar o socket de envio de informações de debug em VSS-SampleStrategys ou VSS-Joysticks (Time Azul)
	void createSendDebugTeam2(vss_debug::Global_Debug*, string addr_client_debug_team2 = "tcp://localhost:5559");
	//! Método responsável por enviar uma nova informação de debug por VSS-SampleStrategys ou VSS-Joysticks (Time Azul)
	void sendDebugTeam2();
	//! Método responsável por criar o socket de recebimento de informações de debug no VSS-Simulator (Time Azul)
	void createReceiveDebugTeam2(vss_debug::Global_Debug*, string addr_server_debug_team2 = "tcp://*:5559");
	//! Método repsonsável por receber uma nova informação de debug (Time Azul)
	void receiveDebugTeam2();

	//! Método que pode ser utilizado para imprimir o pacote de estado recebido/enviado no terminal
	void printState();
	//! Método que pode ser utilizado para imprimir o pacote de comando recebido/enviado no terminal
	void printCommand();
	//! Método que pode ser utilizado para imprimir o pacote de debug recebido/enviado no terminal
	void printDebug();
};

#endif // _INTERFACE_H_
