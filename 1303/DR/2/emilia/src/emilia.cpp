#include "emilia.h"

Emilia::Emilia()
{
  subscriber = nodeHandler.subscribe("SCRUM_meeting", 1000, &Emilia::NewMessageCallback, this);
  msg_counter = 0;
  highlight = "\033[1;31m";
  reset = "\033[0m";
}

void Emilia::NewMessageCallback(const ReportMsg::ConstPtr & msg)
{
  msg_counter++;
  printMsg(msg->content);
}

void Emilia::listenToDoctor()
{
  Rate rate(1);

  while(ok())
  {
    spinOnce();
    rate.sleep();

    if(subscriber.getNumPublishers() == 0)
      shutdown();
  }
}

void Emilia::printMsg(const string & msg_text)
{
  if(msg_counter % 5 == 0)
    cout << highlight << msg_text << reset; 
  else 
    cout << msg_text;

  cout << endl;
}

void Emilia::waitForDoctor() const
{
  while(subscriber.getNumPublishers() == 0)
    sleep(1);
}