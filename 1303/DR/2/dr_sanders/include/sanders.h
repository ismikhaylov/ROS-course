#ifndef DR_SANDERS_H
#define DR_SANDERS_H

#include "ros/ros.h"
#include "gn_tokugava/ReportMsg.h"
#include "ros/package.h"
#include <iostream>
#include <fstream>
#include <string>

using namespace std;
using namespace ros;
using gn_tokugava::ReportMsg;



class DrSanders
{
private:
  NodeHandle nodeHandler;
  Publisher publisher;
  ifstream in;

  void sendMsg(const string & msg_text);
  void printMsg(const string & msg_text);
  void say(const string & msg_text);

public:
  DrSanders();

  bool getReady();
  void tellReport();
  void tellStatus();
  void waitForColleagues() const;
};



#endif // DR_SANDERS_H