#include "tf_lab/bot.h"
#include <string>

using namespace std;

class LostBot: public Bot{
private:
	float 	angle;
	float 	speed;

	void runAround();
	void followSaver();

public:
	LostBot();
	LostBot(string saver_name, string name, string rviz_marker_topic, float x, float y);
	void start();
};