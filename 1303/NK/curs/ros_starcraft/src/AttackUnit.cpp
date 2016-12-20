#include "AttackUnit.h"

AttackUnit::AttackUnit(NodeHandle& node, string unit_type, double x, double y, int frame_count, int attack_frame_count):Unit(node, unit_type, x, y, frame_count){
	this->attack_frame_count = attack_frame_count;

	loadAttackSprites(unit_type, "up");
	loadAttackSprites(unit_type, "down");
	loadAttackSprites(unit_type, "left");
	loadAttackSprites(unit_type, "right");
	loadAttackSprites(unit_type, "up_left");
	loadAttackSprites(unit_type, "up_right");
	loadAttackSprites(unit_type, "down_left");
	loadAttackSprites(unit_type, "down_right");
	attack = false;
	input_sub = this->node->subscribe(input_topic, 1000, &AttackUnit::inputReaction, this);
}

void AttackUnit::loadAttackSprites(string& unit_type, string dir){
	for(int i = 0; i < attack_frame_count; i++){
		Texture* t = StarCraft::Core::getInstance().getTexture(unit_type, "/attack_" + dir, i);
		attack_sprites.push_back(t);
	}
}

void AttackUnit::inputReaction(const geometry_msgs::Twist::ConstPtr& msg){
	if(direction != death){
		direction = death;
		attack = false;
		curr_sprite = 0;
	}
}

Sprite AttackUnit::getSprite(){
	if(attack){
		int id = direction*attack_frame_count + curr_sprite;
		Sprite s;
		s.setTexture(*attack_sprites[id]);
		s.setPosition(x, y);
		return s;
	}else{
		return Unit::getSprite();
	}
}

double AttackUnit::getRightBoard(){
	if(attack){
		if(curr_sprite >= attack_frame_count) curr_sprite = 0;
		int id = direction*attack_frame_count + curr_sprite;
		double res = x + attack_sprites[id]->getSize().x;
		return res;
	}else
		return Unit::getRightBoard();
}

double AttackUnit::getBottomBoard(){
	if(attack){
		int id = direction*attack_frame_count + curr_sprite;
		return y + attack_sprites[id]->getSize().y;
	}else
		return Unit::getBottomBoard();
}

void AttackUnit::moveTo(double x, double y){
	attack = false;
	Unit::moveTo(x, y);
}

bool AttackUnit::isInAttackZone(Point p){
	double attack_rad = 4*sqrt(getFrameWidth()) + 2*Configs::attack_radius;
	double center_x = Unit::getCenterX();
	double center_y = Unit::getCenterY();

	double x_right = p.x + p.w;
	double x_left = p.x;
	double y_top = p.y;
	double y_bottom = p.y + p.w;

	bool isLT = (pow(x_left - center_x,2) + pow(y_top - center_y, 2)) <= pow(attack_rad,2);
	bool isRT = (pow(x_right - center_x,2) + pow(y_top - center_y, 2)) <= pow(attack_rad,2);
	bool isLB = (pow(x_left - center_x,2) + pow(y_bottom - center_y, 2)) <= pow(attack_rad,2);
	bool isRB = (pow(x_right - center_x,2) + pow(y_bottom - center_y, 2)) <= pow(attack_rad,2);

	return isLT || isRT || isLB || isRB;
}

void AttackUnit::doAttack(string target){
	if(direction != death){
		attack = true;

		target_pub = node->advertise<geometry_msgs::Twist>(target + "/input", 1000);
		geometry_msgs::Twist msg;
		target_pub.publish(msg);

		broadcastPose();
	}else{
		attack = false;
	}

	nextFrame();
}

void AttackUnit::nextFrame(){
	if(attack){
		dead = direction == death && curr_sprite == attack_frame_count-1;
		if(!dead)
			curr_sprite = (curr_sprite + 1) % attack_frame_count;

		if(direction == death){
			deleteSelf();
		}
	}else{
		Unit::nextFrame();
	}
}