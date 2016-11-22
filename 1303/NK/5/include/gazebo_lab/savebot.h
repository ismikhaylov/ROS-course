#include "gazebo_lab/bot.h"
#include <string>

using namespace std;

class SaveBot: public Bot{
private:
	float 	angle_speed;
	float	speed;
	float	exit_x;
	float	exit_y;

	void goToFind();
	void goBack();

public:
	SaveBot();
	SaveBot(string my_name, string lost_name, float x, float y, float w);
	void start();
};