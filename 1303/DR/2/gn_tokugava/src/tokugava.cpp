#include "tokugava.h"

GnTokugava::GnTokugava()
{
  subscriber = nodeHandler.subscribe("SCRUM_meeting", 1000, &GnTokugava::NewMessageCallback, this);
  msg_counter = 0;
  highlight = "\033[1;34m";
  reset = "\033[0m";
}

void GnTokugava::NewMessageCallback(const ReportMsg::ConstPtr & msg)
{
  msg_counter++;
  printMsg(msg->content);
}

void GnTokugava::listenToDoctor()
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

void GnTokugava::printMsg(const string & msg_text)
{
  if(msg_counter % 3 == 0)
    cout << highlight << msg_text << reset; 
  else 
    cout << msg_text;

  cout << endl;
}

void GnTokugava::waitForDoctor() const
{
  while(subscriber.getNumPublishers() == 0)
    sleep(1);
}