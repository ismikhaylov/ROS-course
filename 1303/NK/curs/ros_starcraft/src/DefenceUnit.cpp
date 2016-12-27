#include "DefenceUnit.h"

DefenceUnit::DefenceUnit(NodeHandle& node, string unit_type, double x, double y, int frame_count, int defence_frame_count):Unit(node, unit_type, x, y, frame_count){
	this->defence_frame_count = defence_frame_count;
	loadDefenceSprites(unit_type, "up");
	loadDefenceSprites(unit_type, "down");
	loadDefenceSprites(unit_type, "left");
	loadDefenceSprites(unit_type, "right");
	loadDefenceSprites(unit_type, "up_left");
	loadDefenceSprites(unit_type, "up_right");
	loadDefenceSprites(unit_type, "down_left");
	loadDefenceSprites(unit_type, "down_right");

	defence = false;
	defence_cast = Configs::shield_cast_time;
	defence_cooldown = Configs::shield_cooldowm_time;

	protect_id = 0;

	input_sub = this->node->subscribe(input_topic, 1000, &DefenceUnit::inputReaction, this);
}

void DefenceUnit::loadDefenceSprites(string& unit_type, string dir){
	for(int i = 0; i < defence_frame_count; i++){
		Texture* t = StarCraft::Core::getInstance().getTexture(unit_type, "/defence_" + dir, i);
		defence_sprites.push_back(t);
	}
}

Sprite DefenceUnit::getSprite(){
	if(defence){
		int id = direction*defence_frame_count + curr_sprite;
		Sprite s;
		s.setTexture(*defence_sprites[id]);
		s.setPosition(x, y);
		return s;
	}else{
		return Unit::getSprite();
	}
}

void DefenceUnit::moveTo(double x, double y){
	defence = false;
	Unit::moveTo(x, y);
}

void DefenceUnit::inputReaction(const geometry_msgs::Twist::ConstPtr& msg){
	if(direction != death){
		doDefence();
		if(!defence){
			direction = death;
			curr_sprite = 0;
		}
	}
}

void DefenceUnit::doDefence(){
	if(defence_cast > 0){
		defence_cooldown = Configs::shield_cooldowm_time;
		defence_cast -= Configs::shield_discarge;
		defence = true;
	}else if(defence_cooldown <=0){
		defence_cast = Configs::shield_cast_time;
		defence = true;
	}else {
		defence_cooldown -= Configs::shield_discarge;
		defence = false;
	}
	nextFrame();
}

bool DefenceUnit::isInDefenceZone(Point p){
	double center_x = Unit::getCenterX();
	double center_y = Unit::getCenterY();

	double x_right = p.x + p.w;
	double x_left = p.x;
	double y_top = p.y;
	double y_bottom = p.y + p.w;

	bool isLT = (pow(x_left - center_x,2) + pow(y_top - center_y, 2)) <= pow(Configs::defence_radius,2);
	bool isRT = (pow(x_right - center_x,2) + pow(y_top - center_y, 2)) <= pow(Configs::defence_radius,2);
	bool isLB = (pow(x_left - center_x,2) + pow(y_bottom - center_y, 2)) <= pow(Configs::defence_radius,2);
	bool isRB = (pow(x_right - center_x,2) + pow(y_bottom - center_y, 2)) <= pow(Configs::defence_radius,2);

	return isLT || isRT || isLB || isRB;
}

void DefenceUnit::nextFrame(){
	if(defence){
		dead = direction == death && curr_sprite == defence_frame_count-1;
		if(!dead)
			curr_sprite = (curr_sprite + 1) % defence_frame_count;

		if(direction == death){
			deleteSelf();
		}
	}else{
		Unit::nextFrame();
	}
}

double DefenceUnit::getRightBoard(){
	if(defence){
		int id = direction*defence_frame_count + curr_sprite;
		return x + defence_sprites[id]->getSize().x;
	}else
		return Unit::getRightBoard();
}

double DefenceUnit::getBottomBoard(){
	if(defence){
		int id = direction*defence_frame_count + curr_sprite;
		return y + defence_sprites[id]->getSize().y;
	}else
		return Unit::getBottomBoard();
}