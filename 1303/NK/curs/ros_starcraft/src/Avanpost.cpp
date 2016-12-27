#include "Avanpost.h"

Avanpost::Avanpost(NodeHandle& node, string unit_type, double x, double y, int frame_count):Unit(node, unit_type, x, y, frame_count){
	underZerg = true;
	startCreation();
}

void Avanpost::startCreation(){
	zerg_create_progress = Configs::zerg_create_time;
}

void Avanpost::progress(){
	zerg_create_progress -= Configs::zerg_create_discarge;
}

bool Avanpost::isInControlZone(Point p){
	double center_x = Unit::getCenterX();
	double center_y = Unit::getCenterY();

	double x_left = p.x;
	double y_top = p.y;
	double x_right = p.x + p.w;
	double y_bottom = p.y + p.w;

	bool isLT = (pow(x_left - center_x,2) + pow(y_top - center_y, 2)) <= pow(Configs::avanpost_control_radius,2);
	bool isRT = (pow(x_right - center_x,2) + pow(y_top - center_y, 2)) <= pow(Configs::avanpost_control_radius,2);
	bool isLB = (pow(x_left - center_x,2) + pow(y_bottom - center_y, 2)) <= pow(Configs::avanpost_control_radius,2);
	bool isRB = (pow(x_right - center_x,2) + pow(y_bottom - center_y, 2)) <= pow(Configs::avanpost_control_radius,2);

	return isLT || isRT || isLB || isRB;
}