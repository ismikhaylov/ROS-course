#include "gazebo_lab/bot.h"
#include <string>

using namespace std;

class LostBot: public Bot{
private:
	float 	angle_speed;
	float 	angle;
	float 	speed;

	void runAround();
	void followSaver();

public:
	LostBot();
	LostBot(string my_name, string saver_name, float x, float y, float w);
	void start();
};