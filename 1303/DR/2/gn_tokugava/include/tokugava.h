#ifndef GN_TOKUGAVA_H
#define GN_TOKUGAVA_H

#include "ros/ros.h"
#include "gn_tokugava/ReportMsg.h"
#include <iostream>
#include <string>

using namespace std;
using namespace ros;
using gn_tokugava::ReportMsg;



class GnTokugava
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
  GnTokugava();
  void listenToDoctor();
  void waitForDoctor() const;
};



#endif // GN_TOKUGAVA_H