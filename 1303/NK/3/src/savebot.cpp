#include "tf_lab/savebot.h"


SaveBot::SaveBot(){
	SaveBot("lost", "saver", "saver_marker", 0, 0);
}

SaveBot::SaveBot(string saver_name, string name, string rviz_marker_topic, float x, float y)
	:Bot(saver_name, name, rviz_marker_topic, x, y){
	exit_x = x;
	exit_y = y;
	speed = 0.15;
}

void SaveBot::goToFind(){
	updatePartnerPos();

	float p_x = partner_pos.getOrigin().x();
	float p_y = partner_pos.getOrigin().y();

	float m_x = getX();
	float m_y = getY();

	int signX = m_x < p_x ? 1 : -1;
	int signY = m_y < p_y ? 1 : -1;

	move(m_x + signX*speed, m_y + signY*speed);
}

void SaveBot::goBack(){
    float m_x = getX();
	float m_y = getY();

	if( (pow(exit_x - m_x,2) + pow(exit_y - m_y, 2)) <= pow(delta,2) ){
		move(m_x, m_y);
	} else{
		int signX = m_x < exit_x ? 1 : -1;
		int signY = m_y < exit_y ? 1 : -1;

		move(m_x + signX*speed, m_y + signY*speed);
	}
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
		rate.sleep();
	}
}