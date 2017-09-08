/*
 *Este código é uma versão modificada do código original presente no VSS-Samples.
 *É necessária a presença dos outros arquivos do código de fonte deste para o correto funcionamento.
 *Este código não foi destinado a ser exibido, por isto carece de comentários.
 *Porém pareceu ao grupo de Navegação coerente incluí-lo já que sua execução foi mostrada durante a
 *Apresentação Final.

 * This file is part of the VSS-SampleStrategy project.
 *
 * This Source Code Form is subject to the terms of the GNU GENERAL PUBLIC LICENSE,
 * v. 3.0. If a copy of the GPL was not distributed with this
 * file, You can obtain one at http://www.gnu.org/licenses/gpl-3.0/.
 */

#include "strategy.h"
#include <math.h>

Strategy::Strategy(){
    main_color = "yellow";
    is_debug = false;
    real_environment = false;
	robot_radius = 8.0;
	distance_to_stop = 5.0;
	changePose = true;
	srand(time(NULL));
}

void Strategy::init(string main_color, bool is_debug, bool real_environment, string ip_receive_state, string ip_send_debug, string ip_send_command, string name){
	init_sample(main_color, is_debug, real_environment, ip_receive_state, ip_send_debug, ip_send_command, name);
	loop();
}

void Strategy::loop(){
	ros::Rate r(60);
	while(true && ros::ok()) {
		// DON'T REMOVE receive_data();
		receive_state();
		// DON'T REMOVE receive_Data();'

		calc_strategy();

		if(!real_environment) {
			// DON'T REMOVE send_data();
			send_commands();
			// DON'T REMOVE send_data();
		}else{
			// Put your transmission code here
		}

		// DON'T REMOVE
		if(is_debug)
			send_debug();
		// DON'T REMOVE'
		ros::spinOnce();
		r.sleep();
	}
}

int defineGoleiro (btVector3 R1, btVector3 R2, btVector3 R3){
	int retorno;
	btVector3 gol;
	gol.x = 10;
	gol.y = 65;
	if (distancePoint(R1, gol) < distancePoint(R2, gol) && distancePoint(R1, gol) < distancePoint(R3, gol))
		retorno = 0;
	else if (distancePoint(R2, gol) < distancePoint(R1, gol) && distancePoint(R2, gol) < distancePoint(R3, gol))
		retorno = 1;
	else
		retorno = 2;
	return retorno;
}

int defineArtilheiro (btVector3 R1, btVector3 R2, btVector3 R3, btVector3 bola){
	int retorno;
	if (distancePoint(R1, bola) < distancePoint(R2, bola) && distancePoint(R1, bola) < distancePoint(R3, bola))
		retorno = 0;
	else if (distancePoint(R2, bola) < distancePoint(R1, bola) && distancePoint(R2, bola) < distancePoint(R3, bola))
		retorno = 1;
	else
		retorno = 2;
	return retorno;
}

btVector3 posicaoDoArtilheiro (btVector3 bola, btVector3 artilheiro) {
	btVector3 gol, retorno;
	gol.x = 160;
	gol.y = 65;
	float a, b;
	a = (gol.y - bola.y) / (gol.x - bola.x);
	b = gol.y - a*gol.x;
	if(artilheiro.x < bola.x){
		if(artilheiro.y > ((a*artilheiro.x + b) + 10)  || artilheiro.y < ((a*artilheiro.x + b) - 10)){
			retorno.x = artilheiro.x;
			retorno.y = a*artilheiro.x + b;
		}
		else {
			retorno = gol;
		}
	}

	else{
		retorno.x = bola.x - 5.7; //5.7 = diagonal do robo de lado 8cm
		retorno.y =a*retorno.x + b;
	}
	
	//retorno = bola;
	return retorno;
}

btVector3 posicaoDoGoleiro (btVector3 bola, btVector3 goleiro) {
	btVector3 retorno;
	btVector3 gol;
	gol.x = 17.5;
	gol.y = 65;
	if (abs(goleiro.x - gol.x) > 1){
		retorno = gol;
		retorno.z = 270;
	}
	else if (abs(bola.x-goleiro.x) < 47.5) {
		retorno.x = gol.x;
		retorno.y = bola.y;
		retorno.z = 270;
	}
	else {
		retorno = goleiro;
		retorno.z = 270;
	}
	return retorno;
}




void Strategy::calc_strategy(){
	if(changePose){
		changePose = false;
		final.x = state.ball.x;
		final.y = state.ball.y;
		final.z = rand() % 360;
	}
	int goleiro = defineGoleiro (state.robots[0].pose, state.robots[1].pose, state.robots[2].pose);
	int artilheiro = defineArtilheiro (state.robots[0].pose, state.robots[1].pose, state.robots[2].pose, state.ball);

	//if (std::min(std::min(distancePoint(goal, act);, y), z); //Define goleiro
	commands[goleiro] = calc_cmd_to(state.robots[goleiro].pose, posicaoDoGoleiro(state.ball,state.robots[goleiro].pose), distance_to_stop);
	commands[artilheiro] = calc_cmd_to(state.robots[artilheiro].pose, posicaoDoArtilheiro(state.ball,state.robots[artilheiro].pose), distance_to_stop);
	//state.robots[0].pose.show();
	// commands[1]
	// commands[2]
	//debug.robots_final_pose[0] = final;

	for(int i = 0 ; i < 3 ; i++){
		debug.robots_path[i].poses.clear();
	}
	debug.robots_path[0].poses.push_back(state.robots[0].pose);
	debug.robots_path[0].poses.push_back(final);
	debug.robots_path[1].poses.push_back(state.robots[1].pose);
	debug.robots_path[1].poses.push_back(final);
	debug.robots_path[2].poses.push_back(state.robots[2].pose);
	debug.robots_path[2].poses.push_back(final);
}

common::Command Strategy::calc_cmd_to(btVector3 act, btVector3 goal, float distance_to_stop){
	Command cmd;
	float distance_robot_goal;
	float angulation_robot_goal;
	float angulation_robot_robot_goal;

	// Diferença entre angulação do robô e do objetivo
	distance_robot_goal = distancePoint(goal, act);
	angulation_robot_goal = angulation(goal, act);


	angulation_robot_goal -= 180; // 180 if comes from VSS-Simulator
    if(angulation_robot_goal < 0){
    	angulation_robot_goal += 360;
    }

	angulation_robot_robot_goal = act.z - angulation_robot_goal;

	if(angulation_robot_robot_goal > 180){
		angulation_robot_robot_goal -= 360;
	}

	if(angulation_robot_robot_goal < -180){
		angulation_robot_robot_goal += 360;
	}
	
	//cout << angulation_robot_robot_goal << endl;

	// Regras de movimentação
	if(fabs(angulation_robot_robot_goal) <= 135){
		cmd.left = distance_robot_goal - 0.2*(angulation_robot_robot_goal * robot_radius / 2.00);
		cmd.right = distance_robot_goal + 0.2*(angulation_robot_robot_goal * robot_radius / 2.00);
		
		cmd.left *= 0.3;
		cmd.right *= 0.3;
	}else{
		if(angulation_robot_robot_goal >= 0){
			cmd.left = 50;
			cmd.right = -50;
		}else{
			cmd.left = -50;
			cmd.right = 50;
		}
	}

	//cmd.left = 1;
	//cmd.right = -1;
	//cmd.left e cmd.right são PWM (0 a 255 para frente) (256 á 252 para trás)

	/*if(distance_robot_goal < 15.0){
		cmd.left = 0;
		cmd.right = 0;
		changePose = true;
	}*/

	return cmd;
}
