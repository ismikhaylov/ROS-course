#ifndef KERRIGAN_H
#define KERRIGAN_H

#include "AttackUnit.h"

class Kerrigan: public AttackUnit{
public:
	Kerrigan(NodeHandle& node, string unit_type, double x, double y, int frame_count, int attack_frame_count);
	virtual void inputReaction(const geometry_msgs::Twist::ConstPtr& msg);
  	int					protect_id;
};
#endif