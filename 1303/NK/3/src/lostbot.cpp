#include "tf_lab/lostbot.h"


LostBot::LostBot(){
	LostBot("lost", "saver", "lost_marker", 0, 0);
}

LostBot::LostBot(string saver_name, string name, string rviz_marker_topic, float x, float y)
	:Bot(saver_name, name, rviz_marker_topic, x, y){
	angle = 0;
	speed = 0.1;
}

void LostBot::runAround(){
	float x = 2*sin(angle);
    float y = 2*cos(angle);
    move(x, y);
	angle += speed;
}

void LostBot::followSaver(){
	updatePartnerPos();

	float p_x = partner_pos.getOrigin().x();
	float p_y = partner_pos.getOrigin().y();

	float m_x = getX();
	float m_y = getY();

	int signX = m_x < p_x ? 1 : -1;
	int signY = m_y < p_y ? 1 : -1;

	if( (pow(p_x - m_x,2) + pow(p_y - m_y, 2)) <= pow(delta,2) ){
		move(m_x, m_y);
	} else{
		move(m_x + signX*speed, m_y + signY*speed);
	}
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