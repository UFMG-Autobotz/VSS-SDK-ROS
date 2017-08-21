/*
* This file is part of the VSS-Vision project.
*
* This Source Code Form is subject to the terms of the GNU GENERAL PUBLIC LICENSE,
* v. 3.0. If a copy of the GPL was not distributed with this
* file, You can obtain one at http://www.gnu.org/licenses/gpl-3.0/.
*/

#include "common.h"

namespace common{
  void clearSS(stringstream &ss){
    ss.str(string());
    ss.clear();
  }

  string toString(int a){
    stringstream ss;
    ss << a;
    return ss.str();
  }

  string toString(float a){
    stringstream ss;
    ss << a;
    return ss.str();
  }

  string toString(double a){
    stringstream ss;
    ss << a;
    return ss.str();
  }

  string toString(long long int a){
    stringstream ss;
    ss << a;
    return ss.str();
  }

  string toString(bool a){
    string s;
    if(a) s = "1";
    else s = "0";
    return s;
  }

  int toInt(string a){
    int aa;
    stringstream ss;
    ss << a;
    ss >> aa;
    return aa;
  }

  float toFloat(string a){
    float aa;
    stringstream ss;
    ss << a;
    ss >> aa;
    return aa;
  }

  double toDouble(string a){
    double aa;
    stringstream ss;
    ss << a;
    ss >> aa;
    return aa;
  }

  long long int toLongLongInt(string a){
    long long int aa;
    stringstream ss;
    ss << a;
    ss >> aa;
    return aa;
  }

  bool toBool(string a){
    string b = "0";
    if(a.compare(b) == 0) return false;
    else return true;
  }

  string cmdTerminal(string s){
    QProcess process;
    process.start(s.c_str());
    process.waitForFinished(-1); // will wait forever until finished

    QString stdout = process.readAllStandardOutput();
    string result = stdout.toUtf8().constData();
    return result;
  }

  double distancePoint(btVector3 a, btVector3 b){
    return sqrt(((a.x - b.x)*(a.x - b.x)) + ((a.y - b.y)*(a.y - b.y)));
  }

  double distancePoint(Point a, Point b){
    return sqrt(((a.x - b.x)*(a.x - b.x)) + ((a.y - b.y)*(a.y - b.y)));
  }

  Point midpoint(Point a, Point b){
    return Point(((a.x + b.x) / 2.0), ((a.y + b.y) / 2.0));
  }

  Point midpoint(Rect testLabel){
    return Point(testLabel.x + (testLabel.width/2.0), testLabel.y  + (testLabel.height/2.0));
  }

  //! Addendum
  //! --------
  //!
  float angulation(Point a, Point b){
    //! > Estimate angle between two straight lines.
    //! > One line formed by the two points in the function and
    //! > the other line is formed by the point in the center of
    //! > the robot (estimated using midpoint() function) and for
    //! > a point in the pitch where the line must form a ninety
    //! > degree angle with the pitch side.
    return (atan2(a.y - b.y, a.x - b.x) * (180/CV_PI));
  }

  double radian(Point a, Point b){
    return atan2(a.y - b.y, a.x - b.x);
  }

  vss_sdk_ros::global_state State2global_state(State state, ExecConfiguration exec){
    vss_sdk_ros::global_state global_state;
    TableColor c;

    global_state.id = 0;
    global_state.situation = 0;
    global_state.origin = true;

    vss_sdk_ros::s_ball_state *ball_s;
    ball_s->pose.x = state.ball.x;
    ball_s->pose.y = state.ball.y;

    ball_s->v_pose.x = state.v_ball.x;
    ball_s->v_pose.y = state.v_ball.y;

    ball_s->k_pose.x = state.ball_kalman.x;
    ball_s->k_pose.y = state.ball_kalman.y;

    ball_s->k_v_pose.x = state.v_ball_kalman.x;
    ball_s->k_v_pose.y = state.v_ball_kalman.y;

    global_state.ball = ball_s;

    switch(exec.team_color[0]){
      case YELLOW:{
        for(int i = 0 ; i < 3 ; i++){
          vss_sdk_ros::s_robot_state robot_s; // = global_state.add_robots_blue();

          robot_s.pose.x = state.robots[i+3].pose.x;
          robot_s.pose.y = state.robots[i+3].pose.y;
          robot_s.pose.yaw = state.robots[i+3].pose.z;

          robot_s.v_pose.x = state.robots[i+3].v_pose.x;
          robot_s.v_pose.y = state.robots[i+3].v_pose.y;
          robot_s.v_pose.yaw = state.robots[i+3].v_pose.z;

          robot_s.k_pose.x = state.robots_kalman[i+3].pose.x;
          robot_s.k_pose.y = state.robots_kalman[i+3].pose.y;
          robot_s.k_pose.yaw = state.robots_kalman[i+3].pose.z;

          robot_s.k_v_pose.x = state.robots_kalman[i+3].v_pose.x;
          robot_s.k_v_pose.y = state.robots_kalman[i+3].v_pose.y;
          robot_s.k_v_pose.yaw = state.robots_kalman[i+3].v_pose.z;

          if(exec.secundary_color_2[i] != UNKNOWN){
            robot_s.color.r = c.colors.at(exec.secundary_color_2[i]).rgb[0];
            robot_s.color.g = c.colors.at(exec.secundary_color_2[i]).rgb[1];
            robot_s.color.b = c.colors.at(exec.secundary_color_2[i]).rgb[2];

            //qDebug() << exec.secundary_color_2[i];
          }

          global_state.robots_blue[i] = robot_s;
        }

        for(int i = 0 ; i < 3 ; i++){
          vss_sdk_ros::s_robot_state robot_s;

          robot_s.pose.x = state.robots[i].pose.x;
          robot_s.pose.y = state.robots[i].pose.y;
          robot_s.pose.yaw = state.robots[i].pose.z;

          robot_s.v_pose.x = state.robots[i].v_pose.x;
          robot_s.v_pose.y = state.robots[i].v_pose.y;
          robot_s.v_pose.yaw = state.robots[i].v_pose.z;

          robot_s.k_pose.x = state.robots_kalman[i].pose.x;
          robot_s.k_pose.y = state.robots_kalman[i].pose.y;
          robot_s.k_pose.yaw = state.robots_kalman[i].pose.z;

          robot_s.k_v_pose.x = state.robots_kalman[i].v_pose.x;
          robot_s.k_v_pose.y = state.robots_kalman[i].v_pose.y;
          robot_s.k_v_pose.yaw = state.robots_kalman[i].v_pose.z;

          if(exec.secundary_color_1[i] != UNKNOWN){
            robot_s->color.r = c.colors.at(exec.secundary_color_1[i]).rgb[0];
            robot_s->color.g = c.colors.at(exec.secundary_color_1[i]).rgb[1];
            robot_s->color.b = c.colors.at(exec.secundary_color_1[i]).rgb[2];
          }

          global_state.robots_yellow[i] = robot_s;
        }
      }break;
      case BLUE:{
        for(int i = 0 ; i < 3 ; i++){
          vss_sdk_ros::s_robot_state robot_s;

          robot_s.pose.x = state.robots[i].pose.x;
          robot_s.pose.y = state.robots[i].pose.y;
          robot_s.pose.yaw = state.robots[i].pose.z;

          robot_s.v_pose.x = state.robots[i].v_pose.x;
          robot_s.v_pose.y = state.robots[i].v_pose.y;
          robot_s.v_pose.yaw = state.robots[i].v_pose.z;

          robot_s.k_pose.x = state.robots_kalman[i].pose.x;
          robot_s.k_pose.y = state.robots_kalman[i].pose.y;
          robot_s.k_pose.yaw = state.robots_kalman[i].pose.z;

          robot_s.k_v_pose.x = state.robots_kalman[i].v_pose.x;
          robot_s.k_v_pose.y = state.robots_kalman[i].v_pose.y;
          robot_s.k_v_pose.yaw = state.robots_kalman[i].v_pose.z;

          if(exec.secundary_color_1[i] != UNKNOWN){
            robot_s.color.r = c.colors.at(exec.secundary_color_1[i]).rgb[0];
            robot_s.color.g = c.colors.at(exec.secundary_color_1[i]).rgb[1];
            robot_s.color.b = c.colors.at(exec.secundary_color_1[i]).rgb[2];

            //qDebug() << exec.secundary_color_1[0];
          }

          global_state.robots_blue[i] = robot_s;
        }

        for(int i = 0 ; i < 3 ; i++){
          vss_sdk_ros::s_robot_state robot_s;// = global_state.add_robots_yellow();

          robot_s.pose.x = state.robots[i+3].pose.x;
          robot_s.pose.y = state.robots[i+3].pose.y;
          robot_s.pose.yaw = state.robots[i+3].pose.z;

          robot_s.v_pose.x = state.robots[i+3].v_pose.x;
          robot_s.v_pose.y = state.robots[i+3].v_pose.y;
          robot_s.v_pose.yaw = state.robots[i+3].v_pose.z;

          robot_s.k_pose.x = state.robots_kalman[i+3].pose.x;
          robot_s.k_pose.y = state.robots_kalman[i+3].pose.y;
          robot_s.k_pose.yaw = state.robots_kalman[i+3].pose.z;

          robot_s.k_v_pose.x = state.robots_kalman[i+3].v_pose.x;
          robot_s.k_v_pose.y = state.robots_kalman[i+3].v_pose.y;
          robot_s.k_v_pose.yaw = state.robots_kalman[i+3].v_pose.z;

          if(exec.secundary_color_2[i] != UNKNOWN){
            robot_s.color.r = c.colors.at(exec.secundary_color_2[i]).rgb[0];
            robot_s.color.g = c.colors.at(exec.secundary_color_2[i]).rgb[1];
            robot_s.color.b = c.colors.at(exec.secundary_color_2[i]).rgb[2];
          }

          global_state.robots_yellow[i] = robot_s;
        }
      }break;
    }

    return global_state;
  }
}
