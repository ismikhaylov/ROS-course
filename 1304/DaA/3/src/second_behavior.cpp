#include "second_behavior.h"
#include <ctime>

SecondBehavior::SecondBehavior(Bot *bot, string target, string frame_id) {
	this->bot = bot;
	this->target = target;
	this->frame_id = frame_id;
	srand(0);
}

void SecondBehavior::run() {
	run_around();
}

void SecondBehavior::run_around() {
	Rate rate(10);
	ROS_INFO("***STARTING WALKING***");
	while(bot->getNode()->ok()) {
		Vector3 target_pos;
		if (!listen_position(frame_id, target, target_listener, target_pos)) {
			ROS_INFO("***WAITING FOR FIRST BOT***");
			move(this->bot, this->bot->getX() + (double)(rand() % 10), this->bot->getY() + (double)(rand() % 10));
			ROS_INFO("Moving to position [%f; %f]", this->bot->getX(), this->bot->getY());
			rate.sleep();
			continue;
		}
		if (collision(bot->getX(), bot->getY(), target_pos.x(), target_pos.y(), 0.05)) {
			ROS_INFO("***COLLISION***");
			run_back();
			return;
		}
		move(this->bot, this->bot->getX() + (double)(rand() % 3 * pow(-1, 1+rand()%2)), this->bot->getY() + (double)(rand() % 3 * pow(-1, 1+rand()%2)));
		ROS_INFO("Moving to position [%f; %f]", this->bot->getX(), this->bot->getY());
		rate.sleep();
	}
}

void SecondBehavior::run_back() {
	ROS_INFO("***FOLLOWING***");
	Rate rate(10);
	while(bot->getNode()->ok()) {
		Vector3 target_pos;
		if (!listen_position(frame_id, target, target_listener, target_pos)) {
			rate.sleep();
			ROS_INFO("***WAITING FOR FIRST BOT***");
			continue;
		}
		move(this->bot, target_pos.x(), target_pos.y());
		//ROS_INFO("Moving to position [%f; %f]", this->bot->getX(), this->bot->getY());
		rate.sleep();
	}
}