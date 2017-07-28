/*The MIT License (MIT)

Copyright (c) 2016 Lucas Borsatto Simão

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
*/

#include "Simulator.h"

// para evitar erros de ligaçao:
//https://www.codeproject.com/Articles/48575/How-to-define-a-template-class-in-a-h-file-and-imp
#include "interface.cpp"

Simulator::Simulator(){
  // inicializa ros
  int c = 1;
  char *v = "";
  ros::init(c, &v, "simulator");

  contDebug = 0;
  stratStep = 0;
  loopBullet = 0;
  numRobotsTeam = NUM_ROBOTS_TEAM;

  gameState = new GameState();
  caseWorld = NONE;
  runningPhysics = false;

  for(int i = 0 ; i < 6 ; i++){
    Command cmd(0, 0);
    commands.push_back(cmd);
  }

  status_team_1 = status_team_2 = -1;

  count_situation = 0;
  situation_team1 = situation_team2 = 0;

  fast_travel = false;
  qtd_of_goals = 10;

  finish_match = false;

  goals_team_1 = 0;
  goals_team_2 = 0;
  has_new_name_team_1 = false;
  has_new_name_team_2 = false;
}

void Simulator::runSimulator(int argc, char *argv[], ModelStrategy *stratBlueTeam, ModelStrategy *stratYellowTeam, bool fast_travel, int qtd_of_goals, bool develop_mode){
  this->fast_travel = fast_travel;
  this->qtd_of_goals = qtd_of_goals;
  this->develop_mode = develop_mode;

  if(!fast_travel){
    timeStep = 1.f/60.f;
    handTime = 1.f;
  }else{
    timeStep = 1.f/60.f;
    handTime = 15.f;
  }

  int numTeams = 0;
  if(stratBlueTeam) {
    this->strategies.push_back(stratBlueTeam);
    numTeams++;
  }

  if(stratYellowTeam){
    this->strategies.push_back(stratYellowTeam);
    numTeams++;
  }

  if(numTeams == 0){
    cout << "You must set a strategy to run the simulator!" << endl;
    exit(1);
  }

  physics = new Physics(numTeams);

  vector<RobotPhysics*> gRobots = physics->getAllRobots();

  for(int i = 0; i < physics->getNumTeams();i++){
    vector<RobotStrategy*> robotStrategiesTeam;
    for(int j = 0; j < numRobotsTeam;j++){
      RobotStrategy* robotStrategy = new RobotStrategy(0);
      robotStrategiesTeam.push_back(robotStrategy);
    }
    gameState->robotStrategiesTeam = robotStrategiesTeam;
    gameState->robotStrategiesAdv = robotStrategiesTeam;
  }

  thread_physics = new thread(bind(&Simulator::runPhysics, this));
  thread_strategies = new thread(bind(&Simulator::runStrategies, this));
  thread_receive_team1 = new thread(bind(&Simulator::runReceiveTeam1, this));
  thread_receive_team2 = new thread(bind(&Simulator::runReceiveTeam2, this));

  thread_physics->join();
  thread_strategies->join();
  thread_receive_team1->join();
  thread_receive_team2->join();

  report.show();
}


void Simulator::runReceiveTeam1(){
  // YELLOW
  Interface<vss_sdk::Global_Commands> interface;
  interface.createReceive(&global_commands_team_1, "commandsYellow");

  interface.receive();

  //TODO dar ritmo para o loop
  while(!finish_match && ros::ok()){

    if(status_team_1 == -1){
      status_team_1 = 0;
      cout << "---Time amarelo conectado---" << endl;
    }

    situation_team1 = global_commands_team_1.situation;
    for(int i = 0 ; i < global_commands_team_1.robot_commands.size() ; i++){
      commands.at(i) = Command((float)global_commands_team_1.robot_commands[i].left_vel+0.001, (float)global_commands_team_1.robot_commands[i].right_vel+0.001);
    }

    if(global_commands_team_1.name.length()) {
      name_team_1 = global_commands_team_1.name;
      has_new_name_team_1 = true;
    }
  }
}

void Simulator::runReceiveTeam2(){
  // BLUE
  Interface <vss_sdk::Global_Commands> interface;
  interface.createReceive(&global_commands_team_2, "commandsBlue");

  interface.receive();

  while(!finish_match && ros::ok()){

    if(status_team_2 == -1){
      status_team_2 = 0;
      cout << "---Time azul conectado---" << endl;
    }

    situation_team2 = global_commands_team_2.situation;
    for(int i = 0 ; i < global_commands_team_2.robot_commands.size() ; i++){
      commands.at(i+3) = Command((float)global_commands_team_2.robot_commands[i].left_vel+0.001, (float)global_commands_team_2.robot_commands[i].right_vel+0.001);
    }

    if(global_commands_team_2.name.length()) {
      name_team_2 = global_commands_team_2.name;
      has_new_name_team_2 = true;
    }
  }
}

void Simulator::runSender(Interface<vss_sdk::Global_State> *interface){
  global_state.id = 0;
  global_state.situation = caseWorld;
  global_state.origin = false;

  if(report.total_of_goals_team[0] != goals_team_1){
    goals_team_1 = report.total_of_goals_team[0];
    global_state.goals_yellow = goals_team_1;
  }

  if(report.total_of_goals_team[1] != goals_team_2){
    goals_team_2 = report.total_of_goals_team[1];
    global_state.goals_blue = goals_team_2;
  }

  if(has_new_name_team_1){
    has_new_name_team_1 = false;
    global_state.name_yellow = name_team_1;
  }

  if(has_new_name_team_2){
    has_new_name_team_2 = false;
    global_state.name_blue = name_team_2;
  }

  vss_sdk::s_Ball_State ball_s;
  ball_s.pose.x = physics->getBallPosition().getX();
  ball_s.pose.y = physics->getBallPosition().getZ();

  ball_s.v_pose.x = physics->getBallVelocity().getX();
  ball_s.v_pose.y = physics->getBallVelocity().getZ();

  ball_s.k_pose.x = 0;
  ball_s.k_pose.y = 0;

  ball_s.k_v_pose.x = 0;
  ball_s.k_v_pose.y = 0;

  global_state.ball = ball_s;

  vector<RobotPhysics*> listRobots = physics->getAllRobots();
  for(int i = 0 ; i < 3 ; i++){
    vss_sdk::s_Robot_State robot_s;
    btVector3 posRobot = getRobotPosition(listRobots.at(i+3));
    btVector3 velRobot = getRobotVelocity(listRobots.at(i+3));

    robot_s.pose.x = posRobot.getX();
    robot_s.pose.y = posRobot.getZ();
    float rads = atan2(getRobotOrientation(listRobots.at(i+3)).getZ(),getRobotOrientation(listRobots.at(i+3)).getX());
    robot_s.pose.yaw = rads;

    robot_s.v_pose.x = velRobot.getX();
    robot_s.v_pose.y = velRobot.getZ();
    robot_s.v_pose.yaw = 0;

    robot_s.k_pose.x = 0;
    robot_s.k_pose.y = 0;
    robot_s.k_pose.yaw = 0;

    robot_s.k_v_pose.x = 0;
    robot_s.k_v_pose.y = 0;
    robot_s.k_v_pose.yaw = 0;

    global_state.robots_blue[i] = robot_s;
  }

  for(int i = 0 ; i < 3 ; i++){
    vss_sdk::s_Robot_State robot_s;
    btVector3 posRobot = getRobotPosition(listRobots.at(i));
    btVector3 velRobot = getRobotVelocity(listRobots.at(i));

    robot_s.pose.x = posRobot.getX();
    robot_s.pose.y = posRobot.getZ();
    float rads = atan2(getRobotOrientation(listRobots.at(i)).getZ(),getRobotOrientation(listRobots.at(i)).getX());
    robot_s.pose.yaw = rads;

    robot_s.v_pose.x = velRobot.getX();
    robot_s.v_pose.y = velRobot.getZ();
    robot_s.v_pose.yaw = 0;

    robot_s.k_pose.x = 0;
    robot_s.k_pose.y = 0;
    robot_s.k_pose.yaw = 0;

    robot_s.k_v_pose.x = 0;
    robot_s.k_v_pose.y = 0;
    robot_s.k_v_pose.yaw = 0;

    global_state.robots_yellow[i] = robot_s;
  }

  interface->send();
}

void Simulator::runPhysics(){
  int subStep = 1;
  float standStep = 1.f/60.f;

  arbiter.allocPhysics(physics);
  arbiter.allocReport(&report);

  Interface <vss_sdk::Global_State> interface;
  interface.createSend(&global_state, "state");

  // REVIEW por enquanto estou inicializando o vetor de robos e de bolas assim
  // porque acho que o viewer pode mandar mais de 3 robos e mais de uma bolas
  // verificar se é possivel so definir o tamanho no .msg

  while(!finish_match && ros::ok()){
    usleep(1000000.f*timeStep/handTime);

    //physics->setBallVelocity(btVector3(0.1, 0, 0));
    loopBullet++;
    //cout << "--------Ciclo Atual:\t" << loopBullet << "--------" << endl;
    if(gameState->sameState){
      physics->stepSimulation(timeStep,subStep,standStep);
      gameState->sameState = false;

      report.qtd_of_steps++;
    }

    updateReport();
    runningPhysics = true;

    arbiter.checkWorld();

    if(!develop_mode){
      if(report.total_of_goals_team[0] >= qtd_of_goals || report.total_of_goals_team[1] >= qtd_of_goals || report.qtd_of_steps > 3500*qtd_of_goals){
        finish_match = true;
      }
    }

    runSender(&interface);
  }
}

void Simulator::updateReport(){
  vector<BulletObject*> listRobots = physics->getAllBtRobots();

  float minDist = 9999;
  int idDist = -1;
  for(int i = 0; i < listRobots.size(); i++){
    if(listRobots.at(i)->hitRobot){
      if(i >= listRobots.size()/2){
        int id = i- listRobots.size()/2;
        report.collisions_in_high_speed_team[1][id]++;
      }else{
        report.collisions_in_high_speed_team[0][i]++;
      }
    }

    btTransform transTemp;
    listRobots.at(i)->body->getMotionState()->getWorldTransform(transTemp);

    btVector3 rbPos = transTemp.getOrigin();
    btVector3 ballPos = physics->getBallPosition();

    btVector3 dBallRbDist = rbPos - ballPos;

    btVector3 velRobot = listRobots.at(i)->body->getLinearVelocity()*timeStep;
    //cout << "traveled:\t" << velRobot.length() << endl;
    float modDist = dBallRbDist.length();

    if(i >= listRobots.size()/2){
      int id = i- listRobots.size()/2;
      report.travelled_distance_team[1][id] += velRobot.length();
      //cout << "testeeeee-1" << endl;
    }else{
      report.travelled_distance_team[0][i] += velRobot.length();
    }


    if(minDist > modDist){
      minDist = modDist;
      idDist = i;
    }
  }

  if(idDist > listRobots.size()/2){
    report.ball_possession_team[1]++;
  }else{
    report.ball_possession_team[0]++;
  }
}

void Simulator::runStrategies(){
  btVector3 posTargets[] = {btVector3(SIZE_WIDTH,0,SIZE_DEPTH/2),btVector3(0,0,SIZE_DEPTH/2)};
  int attackDir = 0;
  int framesSec = (int)(1/timeStep);

  for(int i = 0; i < physics->getNumTeams();i++){
    if(posTargets[i].getX() > 0)  attackDir = 1;
    else attackDir = -1;
    strategies[i]->setAttackDir(attackDir);
    strategies[i]->setFramesSec(framesSec);

    for(int j = 0; j < numRobotsTeam;j++){
      int id = i*numRobotsTeam + j;
      physics->getAllRobots()[id]->setTimeStep(timeStep);
    }

  }

  while(!finish_match && ros::ok()){
    usleep(1000000.f*timeStep/handTime);

    if(!gameState->sameState){
      updateWorld();

      if(strategies.size() > 0){
        btVector3 ballPos = calcRelativePosition(physics->getBallPosition(),strategies[0]->getAttackDir());
        calcRelativeWorld(gameState->robotStrategiesTeam,strategies[0]->getAttackDir());

        strategies[0]->runStrategy(gameState->robotStrategiesTeam,gameState->robotStrategiesTeam,ballPos);
        if(strategies.size() == 2){
          btVector3 ballPos = calcRelativePosition(physics->getBallPosition(),strategies[1]->getAttackDir());
          calcRelativeWorld(gameState->robotStrategiesAdv,strategies[1]->getAttackDir());
          calcRelativeWorld(gameState->robotStrategiesTeam,strategies[0]->getAttackDir());
          strategies[1]->runStrategy(gameState->robotStrategiesAdv,gameState->robotStrategiesTeam,ballPos);
        }
      }else{
        cout << "You must set a strategy to run the simulator!\n" << endl;
        exit(0);
      }

      for(int i = 0; i < physics->getNumTeams(); i++){
        for(int j = 0; j < numRobotsTeam; j++){
          int id = i*numRobotsTeam + j;
          if(strategies[i]->getAttackDir() == 1){
            //cout << id << endl;
            float command[2] = { commands.at(id).left, commands.at(id).right };
            //if(id == 0)
            //cout << command[0] << " - " << command[1] << endl;

            //command[1] = strategies[i]->getRobotStrategiesTeam()[j]->getCommand()[1];
            //command[0] = strategies[i]->getRobotStrategiesTeam()[j]->getCommand()[0];

            physics->getAllRobots()[id]->updateRobot(command);
          }
          else{
            float invCommand[2] = { commands.at(id).left, commands.at(id).right };

            //invCommand[0] = strategies[i]->getRobotStrategiesTeam()[j]->getCommand()[1];
            //invCommand[1] = strategies[i]->getRobotStrategiesTeam()[j]->getCommand()[0];

            physics->getAllRobots()[id]->updateRobot(invCommand);
          }
        }
      }

      gameState->sameState = true;
    }
  }
}

void Simulator::updateWorld(){
  vector<RobotPhysics*> gRobots = physics->getAllRobots();

  for(int i = 0; i < physics->getNumTeams();i++){
    vector<RobotStrategy*> robotStrategiesTeam;
    for(int j = 0; j < numRobotsTeam;j++){
      RobotStrategy* robotStrategy;
      int idRobot = i*numRobotsTeam+j;
      robotStrategy = updateLocalPhysics(j, gRobots[idRobot]);
      robotStrategiesTeam.push_back(robotStrategy);
    }
    if(i == 0)  gameState->robotStrategiesTeam = robotStrategiesTeam;
    else    gameState->robotStrategiesAdv = robotStrategiesTeam;
  }
}

RobotStrategy* Simulator::updateLocalPhysics(int id, RobotPhysics* physicRobot){
  RobotStrategy* robotStrategy = new RobotStrategy(id);

  btTransform  transTemp;
  physicRobot->getRigidBody()->getMotionState()->getWorldTransform(transTemp);
  robotStrategy->setPosition(transTemp.getOrigin());

  btVector3 forwardVec = physicRobot->getRaycast()->getForwardVector();

  robotStrategy->setLocalFront(forwardVec);

  return robotStrategy;
}

void Simulator::calcRelativeWorld(vector<RobotStrategy*> robotStrategiesTeam,int attackDir){
  for(int i = 0; i < robotStrategiesTeam.size(); i++){
    robotStrategiesTeam[i]->setPosition(calcRelativePosition(robotStrategiesTeam[i]->getPosition(),attackDir));
    robotStrategiesTeam[i]->setLocalFront(attackDir*robotStrategiesTeam[i]->getLocalFront());
    robotStrategiesTeam[i]->setLocalRight(attackDir*robotStrategiesTeam[i]->getLocalRight());
  }
}

btVector3 Simulator::calcRelativePosition(btVector3 absPos, int attackDir){
  float relX = absPos.getX();
  //cout << attackDir;
  float relZ = absPos.getZ();
  if(attackDir == -1){

    relZ = SIZE_DEPTH - absPos.getZ();
    relX = SIZE_WIDTH - absPos.getX();
  }
  return btVector3(relX,0,relZ);
}

btVector3 Simulator::getRobotPosition(RobotPhysics* robot){
  btTransform  transTemp;
  robot->getRigidBody()->getMotionState()->getWorldTransform(transTemp);
  return transTemp.getOrigin();
}

btVector3 Simulator::getRobotOrientation(RobotPhysics* robot){
  btVector3 forwardVec = robot->getRaycast()->getForwardVector();
  return forwardVec;
}

btVector3 Simulator::getRobotVelocity(RobotPhysics* robot){
  return robot->getRigidBody()->getLinearVelocity();
}
