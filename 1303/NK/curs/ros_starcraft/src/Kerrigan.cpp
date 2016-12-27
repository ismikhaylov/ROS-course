#include "Kerrigan.h"

Kerrigan::Kerrigan(NodeHandle& node, string unit_type, double x, double y, int frame_count, int attack_frame_count):AttackUnit(node, unit_type, x, y, frame_count, attack_frame_count){
	input_sub = this->node->subscribe(input_topic, 1000, &Kerrigan::inputReaction, this);
	protect_id = 0;
}

void Kerrigan::inputReaction(const geometry_msgs::Twist::ConstPtr& msg){}