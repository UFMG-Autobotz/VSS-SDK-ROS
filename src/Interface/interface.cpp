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
  //! global_state é recebido como ponteiro, assim facilitando o envio de novos estados
  msg = message;
  nh.setCallbackQueue(&q);
  pub = nh.advertise<msg_Type>(nodeName, 1000);
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
  sub = nh.subscribe(nodeName, 1000, &Interface::callback, this);
}

template <class msg_Type>
void Interface<msg_Type>::callback(const msg_Type message) {
  *msg = message;
}

//! Esse método deve ser chamado em um loop infinito controlado.
//! O recebimento tratado aqui é não bloqueante
template <class msg_Type>
void Interface<msg_Type>::receive(){
  q.callAvailable(ros::WallDuration()); // callAvailable é usado no lugar de spinOnce porque cada mensagem tem sua propria fila
}
