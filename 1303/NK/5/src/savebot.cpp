#include "gazebo_lab/savebot.h"


SaveBot::SaveBot(){
	SaveBot("saver", "lost", 0, 0, 0);
}

SaveBot::SaveBot(string my_name, string lost_name, float x, float y, float w)
	:Bot(my_name, lost_name, x, y, w){
	exit_x = x;
	exit_y = y;
	speed = 0.1;
	angle_speed = 0.1;
}

void SaveBot::goToFind(){
	updatePartnerPos();

	float p_x = partner_pos.getOrigin().x();
	float p_y = partner_pos.getOrigin().y();

	int signX = 0 < p_x ? 1 : -1;
	int signY = 0 < p_y ? 1 : -1;

	if(signX == 1 && signY == 1) w = -0.3;
	if(signX == -1 && signY == 1) w = -2.36;
	if(signX == -1 && signY == -1) w = 2.36;
	if(signX == 1 && signY == -1) w = 0.3;

	x = x + signX*speed;
	y = y + signY*speed;

   	updatePose();
}

void SaveBot::goBack(){


	if( (pow(exit_x - x,2) + pow(exit_y - x, 2)) > pow(delta,2) ){
		int signX = x < exit_x ? 1 : -1;
		int signY = y < exit_y ? 1 : -1;

		if(signX == 1 && signY == 1) w = -0.3;
		if(signX == -1 && signY == 1) w = -2.36;
		if(signX == -1 && signY == -1) w = 2.36;
		if(signX == 1 && signY == -1) w = 0.3;

		x = x + signX*speed;
		y = y + signY*speed;
	}
   	updatePose();
}

void SaveBot::start(){
	Rate rate(10);
	bool found = false;

	while(node.ok()){
		if(!isMet() && !found){
			goToFind();
		} else{
			found = true;
			goBack();
		}
		/*move(x, y);
		updatePartnerPos();*/
		rate.sleep();
	}
}