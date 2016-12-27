#include "behavior.h"

FirstBehavior::FirstBehavior(Bot *bot, string target_name, string frame_id) {
	this->bot = bot;
	this->target = target;
	this->frame_id = frame_id;
}

void FirstBehavior::run() {
	run_forward();
}

void FirstBehavior::run_forward() {
	Rate rate(10);
	while(node.ok()) {
		Vector3 target_pos = listen_position(frame_id, target, target_listener);
		if (collision(bot->getX(), bot->getY(), target_pos.x(), target_pos.y(), 0.5)) {
			run_back();
			return;
		}
		move(this->bot, target_pos.x(), target_pos.y());
		rate.sleep();
	}
}

void FirstBehavior::run_back() {
	Rate rate(10);
	while(node.ok()) {
		move(this->bot, begin_x, begin_y);
		rate.sleep();
	}
}