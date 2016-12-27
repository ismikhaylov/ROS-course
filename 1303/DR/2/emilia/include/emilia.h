#ifndef EMILIA_H
#define EMILIA_H

#include "ros/ros.h"
#include "gn_tokugava/ReportMsg.h"
#include <iostream>
#include <string>

using namespace std;
using namespace ros;
using gn_tokugava::ReportMsg;



class Emilia
{
private:
  NodeHandle nodeHandler;
  Subscriber subscriber;
  int msg_counter;
  string highlight;
  string reset;

  void NewMessageCallback(const ReportMsg::ConstPtr & msg);
  void printMsg(const string & msg_text);

public:
  Emilia();
  void listenToDoctor();
  void waitForDoctor() const;
};



#endif // EMILIA_H