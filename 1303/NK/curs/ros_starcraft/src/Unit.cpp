#include "Unit.h"

int Unit::global_counter = 0;
vector<string>* Unit::unit_ids = new vector<string>;

Unit::Unit(NodeHandle& node, string unit_type, double x, double y, int frame_count){
	path_to_img = "/home/ckyhc/ros/labs/workspace/src/ros_starcraft/img/";
	this->frame_count = frame_count;

	loadSprites(unit_type, "up");
	loadSprites(unit_type, "down");
	loadSprites(unit_type, "left");
	loadSprites(unit_type, "right");
	loadSprites(unit_type, "up_left");
	loadSprites(unit_type, "up_right");
	loadSprites(unit_type, "down_left");
	loadSprites(unit_type, "down_right");
	loadSprites(unit_type, "death");

	direction = up;
	curr_sprite = 0;
	global_counter ++;
	id = global_counter;

	stringstream ss;
	ss << id;

	id_name = "/" + unit_type + ss.str();
	unit_ids->push_back(id_name);

	this->node = &node;
	input_topic = id_name + "/input";

	this->x = x;
	this->y = y;
	collisions[up] = false;
	collisions[down] = false;
	collisions[left_] = false;
	collisions[right_] = false;
	dead = false;
	broadcastPose();
}

void Unit::loadSprites(string& unit_type, string dir){
	for(int i = 0; i < frame_count; i++){
		Texture* t = StarCraft::Core::getInstance().getTexture(unit_type, dir, i);
		common_sprites.push_back(t);
	}
}

bool Unit::isInZone(double x, double y){
	return (pow(this->x - x,2) + pow(this->y - y, 2)) <= pow(Configs::step_delta,2);
}


void Unit::lookTo(double x, double y){
	if(direction != death){
		const double pi = 3.1415926535897932384;

		double angle = getAngleTo(x, y);

		if(angle < pi/8 || angle >= 15*pi/8)
			direction = right_;

		if(angle >= pi/8 && angle < 3*pi/8)
			direction = down_right;

		if(angle >= 3*pi/8 && angle < 5*pi/8)
			direction = down;

		if(angle >= 5*pi/8 && angle < 7*pi/8)
			direction = down_left;

		if(angle >= 7*pi/8 && angle < 9*pi/8)
			direction = left_;

		if(angle >= 9*pi/8 && angle < 11*pi/8)
			direction = up_left;

		if(angle >= 11*pi/8 && angle < 13*pi/8)
			direction = up;

		if(angle >= 13*pi/8 && angle < 15*pi/8)
			direction = up_right;
	}
}

void Unit::moveTo(double x, double y){
	if(direction != death){
		bool vertical_changed = false;

		const double pi = 3.1415926535897932384;

		double angle = getAngleTo(x, y);
		int dir;

		if(angle < pi/8 || angle >= 15*pi/8)
			dir = right_;

		if(angle >= pi/8 && angle < 3*pi/8)
			dir = down_right;

		if(angle >= 3*pi/8 && angle < 5*pi/8)
			dir = down;

		if(angle >= 5*pi/8 && angle < 7*pi/8)
			dir = down_left;

		if(angle >= 7*pi/8 && angle < 9*pi/8)
			dir = left_;

		if(angle >= 9*pi/8 && angle < 11*pi/8)
			dir = up_left;

		if(angle >= 11*pi/8 && angle < 13*pi/8)
			dir = up;

		if(angle >= 13*pi/8 && angle < 15*pi/8)
			dir = up_right;

		switch (dir){
			case right_: 
				if(!collisions[right_]) applyPose(x, y, dir); break;
			case up_right: 
				if(!collisions[right_] && !collisions[up]) applyPose(x, y, dir); break;
			case up: 
				if(!collisions[up]) applyPose(x, y, dir); break;
			case up_left: 
				if(!collisions[up] && !collisions[left_]) applyPose(x, y, dir); break;
			case left_: 
				if(!collisions[left_]) applyPose(x, y, dir); break;
			case down_left: 
				if(!collisions[down] && !collisions[left_]) applyPose(x, y, dir); break;
			case down: 
				if(!collisions[down]) applyPose(x, y, dir); break;
			case down_right: 
				if(!collisions[down] && !collisions[right_]) applyPose(x, y, dir); break;
		}
		broadcastPose();
	}else{
		nextFrame();
	}
}

void Unit::applyPose(double x, double y, int dir){
	this->x = x;
	this->y = y;
	direction = dir;
	nextFrame();
}

void Unit::nextFrame(){
	dead = direction == death && curr_sprite == frame_count-1;
	if(!dead)
		curr_sprite = (curr_sprite + 1) % frame_count;

	if(direction == death){
		deleteSelf();
	}
}

void Unit::broadcastPose(){
	static tf::TransformBroadcaster br;
	tf::Quaternion q;
	q.setRPY(0.0, 0.0, 0.0);

	tf::Transform transform;
	transform.setOrigin(tf::Vector3(x, y, getFrameWidth()));
	transform.setRotation(q);

	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", id_name));

	/*cout << id_name << ": broadcast" << endl;
	cout << "\tsetOrigin: " << x << " " << y << endl;
	cout << "\tsetRPY: " << getFrameWidth() << " " << getFrameWidth() << endl;*/
}

Sprite Unit::getSprite(){
	int id = direction*frame_count + curr_sprite;
	Sprite s;
	s.setTexture(*common_sprites[id]);
	s.setPosition(x, y);
	return s;
}

bool Unit::isInVisibleZone(Point p){
	double center_x = Unit::getCenterX();
	double center_y = Unit::getCenterY();

	double x_left = p.x;
	double y_top = p.y;
	double x_right = p.x + p.w;
	double y_bottom = p.y + p.w;

	bool isLT = (pow(x_left - center_x,2) + pow(y_top - center_y, 2)) <= pow(Configs::visible_radius,2);
	bool isRT = (pow(x_right - center_x,2) + pow(y_top - center_y, 2)) <= pow(Configs::visible_radius,2);
	bool isLB = (pow(x_left - center_x,2) + pow(y_bottom - center_y, 2)) <= pow(Configs::visible_radius,2);
	bool isRB = (pow(x_right - center_x,2) + pow(y_bottom - center_y, 2)) <= pow(Configs::visible_radius,2);

	/*if(id_name == "/kerrigan1"){
		cout << "Point\n\t" << "(" << p.x << ", " << p.y << ") " << x_right << " " << y_bottom << endl;
		cout << isLT << " " << isRT << " " << isLB << " " << isRB << endl;
	}*/

	return isLT || isRT || isLB || isRB;
}

double Unit::getCenterX(){
	return getLeftBoard() + (getRightBoard() - getLeftBoard())/2;
}

double Unit::getCenterY(){
	return getTopBoard() + (getBottomBoard() - getTopBoard())/2;
}

double Unit::getBottomBoard(){
	int id = direction*frame_count + curr_sprite;
	return y + common_sprites[id]->getSize().y;
}

double Unit::getTopBoard(){
	return y;
}

double Unit::getLeftBoard(){
	return x;
}

double Unit::getRightBoard(){
	//cout << id_name << ": Unit::getRightBoard" << endl;
	int id = direction*frame_count + curr_sprite;
	return x + common_sprites[id]->getSize().x;
}

bool Unit::isInFrame(double x, double y){
	//cout << id_name << " isInFrame start" << endl;

	double x_right = getRightBoard();

	//cout << id_name << " before getLeftBoard" << endl;
	double x_left = getLeftBoard();

	//cout << id_name << " before getTopBoard" << endl;
	double y_top = getTopBoard();

	//cout << id_name << " before getBottomBoard" << endl;
	double y_bottom = getBottomBoard();

	//cout << x << " " << y << endl;
	//cout << x_left << " " << x_right << " " <<  y_top << " " << y_bottom << endl << endl;
	bool res = x > x_left && x <= x_right && y >= y_top && y < y_bottom;
	//cout << id_name << " isInFrame start" << endl;
	return res;
	//return x_left < x && x < x_right && y_top < y && y < y_bottom;
}

double Unit::getFrameHeight(){
	return getBottomBoard() - getTopBoard();
}

double Unit::getFrameWidth(){
	return getRightBoard() - getLeftBoard();
}

double Unit::getAngleTo(double x, double y){
	const double pi = 3.1415926535897932384;
	double dx = x - this->x;
	double dy = y - this->y;

	double angle = atan(dy/dx);

	if (dx < 0) return angle + pi;
	if (dx = 0) return (dy > 0) ? pi/2 : 3*pi/2;
	if (angle < 0) return angle + 2*pi;
	return angle;
}

void Unit::goInDirection(double x, double y){
	if(!isInZone(x, y)){
		double angle = getAngleTo(x, y);
		double dx = cos(angle)*Configs::unit_speed;
		double dy = sin(angle)*Configs::unit_speed;
		/*if(id_name == "/kerrigan1"){
			cout << id_name << ":\n\tgoing to (" << x << ", " << y << ")" << endl;
			cout << "\tnew offset (" << dx << ", " << dy << ")" << endl;
		}*/
		if(fabs(dx) < 1.0) dx = 0.0;
		if(fabs(dy) < 1.0) dy = 0.0;
		//cout << id_name << ": (" << dx << ", " << dy << ")" << endl;
		 moveTo(this->x + dx, this->y + dy);
		//moveTo(dx, dy);
	}
}

void  Unit::deleteSelf(){
	for(int i = 0; i < unit_ids->size(); i++){
		if ((*unit_ids)[i] == id_name){
			//cout << id_name << ": deleted from global list\n";
			unit_ids->erase(unit_ids->begin() + i);
			break;
		}
	}
}

void Unit::inputReaction(const geometry_msgs::Twist::ConstPtr& msg){}