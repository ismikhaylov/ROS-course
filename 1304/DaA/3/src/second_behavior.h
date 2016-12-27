#include "robot.h"

using namespace std;

class SecondBehavior {
protected:
	Bot *bot;
	string target;
	string frame_id;
	TransformListener target_listener;

	void run_around();
	void run_back();
public:
	SecondBehavior(Bot *bot, string target_name, string frame_id);
	void run();
};