#include "tf_lab/bot.h"
#include <string>

using namespace std;

class SaveBot: public Bot{
private:
	float	speed;
	float	exit_x;
	float	exit_y;

	void goToFind();
	void goBack();

public:
	SaveBot();
	SaveBot(string lost_name, string name, string rviz_marker_topic, float x, float y);
	void start();
};