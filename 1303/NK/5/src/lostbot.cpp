#include "gazebo_lab/lostbot.h"
#include "math.h"


LostBot::LostBot(){
	LostBot("lost", "saver", 0, 0, 0);
}

LostBot::LostBot(string my_name, string saver_name, float x, float y, float w)
	:Bot(my_name, saver_name, x, y, w){
	angle_speed = 0.05;
	speed = 0.08;
	angle = 0;
}

void LostBot::runAround(){
	angle =  angle + angle_speed;
	if(angle > 6.28) angle = 0;

    w = angle;
    if(angle > 3.14) w = w - 6.28;
    x = 2*sin(angle);
   	y = 2*cos(angle);
   	updatePose();
}

void LostBot::followSaver(){
	updatePartnerPos();

	float p_x = partner_pos.getOrigin().x();
	float p_y = partner_pos.getOrigin().y();

	if( (pow(p_x,2) + pow(p_y, 2)) > pow(delta,2) ){
		int signX = 0 < p_x ? 1 : -1;
		int signY = 0 < p_y ? 1 : -1;

		if(signX == 1 && signY == 1) w = -0.3;
		if(signX == -1 && signY == 1) w = -2.36;
		if(signX == -1 && signY == -1) w = 2.36;
		if(signX == 1 && signY == -1) w = 0.3;

		x = x + signX*speed;
		y = y + signY*speed;
	}
   	updatePose();
}

void LostBot::start(){
	Rate rate(10);
	bool found = false;

	while(node.ok()){
		if(!isMet() && !found){
			runAround();
		} else{
			found = true;
			followSaver();
		}
		rate.sleep();
	}
}