#include "first_behavior.h"

FirstBehavior::FirstBehavior(Bot *bot, string target, string frame_id) {
	this->bot = bot;
	this->target = target;
	this->frame_id = frame_id;

	begin_x = bot->getX();
	begin_y = bot->getY();
}

void FirstBehavior::run() {
	run_forward();
}

void FirstBehavior::run_forward() {
	Rate rate(10);
	ROS_INFO("***LOOKING FOR SECOND BOT***");
	while(bot->getNode()->ok()) {
		Vector3 target_pos;
		if (!listen_position(frame_id, target, target_listener, target_pos)) {
			ROS_INFO("***WAITING FOR SECOND BOT***");
			rate.sleep();
			continue;
		}
		if (collision(bot->getX(), bot->getY(), target_pos.x(), target_pos.y(), 0.05)) {
			ROS_INFO("***COLLISION***");
			run_back();
			return;
		}
		move(this->bot, target_pos.x(), target_pos.y());
		ROS_INFO("Moving to position [%f; %f]", this->bot->getX(), this->bot->getY());
		rate.sleep();
	}
}

void FirstBehavior::run_back() {
	Rate rate(10);
	ROS_INFO("***GO HOME***");
	while(bot->getNode()->ok()) {
		if (move(this->bot, begin_x, begin_y)) {
			//ROS_INFO("***I AM AT HOME***");
			rate.sleep();
			continue;
		}
		ROS_INFO("Moving to position [%f; %f]", this->bot->getX(), this->bot->getY());
		rate.sleep();
	}
}