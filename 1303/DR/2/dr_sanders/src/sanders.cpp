#include "sanders.h"

DrSanders::DrSanders()
{
  publisher = nodeHandler.advertise<ReportMsg>("SCRUM_meeting", 1000);
}

bool DrSanders::getReady()
{
  string filename = package::getPath("dr_sanders") + "/res/dr_sanders.res";
  in.open(filename.c_str());

  return in;
}

void DrSanders::tellReport()
{
  Rate rate(1);

  while (ok() && !in.eof())
  {
    string msg;
    getline(in, msg);
    say(msg);

    rate.sleep();
  }
}

void DrSanders::tellStatus()
{
  if(in) 
  	say("Итак, я готов сообщить вам о том, как прошла вчерашняя встреча с заказчиком.");
  else 
  	say("Прошу прощения. Я не смог найти свой отчет.");
}

void DrSanders::sendMsg(const string & msg_text)
{
  ReportMsg msg;
  msg.author = "dr_sanders";
  msg.content = msg_text;

  publisher.publish(msg);
}

void DrSanders::printMsg(const string & msg_text)
{
  cout << msg_text << endl;
}

void DrSanders::say(const string & msg_text)
{
  sendMsg(msg_text);
  printMsg(msg_text);
}

void DrSanders::waitForColleagues() const
{
  while(publisher.getNumSubscribers() == 0)
    sleep(1);
}