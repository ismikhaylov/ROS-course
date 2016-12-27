#include "robot.h"

using namespace std;

class FirstBehavior {
protected:
	Bot *bot;
	string target;
	string frame_id;
	TransformListener target_listener;
	float begin_x;
	float begin_y;

	void run_forward();
	void run_back();
public:
	FirstBehavior(Bot *bot, string target_name, string frame_id);
	void run();
};