#include "Shuttle.h"


Shuttle::Shuttle(NodeHandle& node, string unit_type, double x, double y, int frame_count):Unit(node, unit_type, x, y, frame_count){
	land_x = 0.0;
	land_y = 0.0;

	base_x = x;
	base_y = y;

	full = true;
	flight_cooldown = Configs::flight_cooldown_time;
}

bool Shuttle::isInLandingZone(){
	return Unit::isInZone(land_x, land_y);
}

bool Shuttle::isInBaseZone(){
	return Unit::isInZone(base_x, base_y);
}

void Shuttle::goToBase(){
	Unit::goInDirection(base_x, base_y);
}

void Shuttle::goToLand(){
	Unit::goInDirection(land_x, land_y);
}

void Shuttle::putOut(){
	full = false;
	flight_cooldown = Configs::flight_cooldown_time;
}

void Shuttle::wasteTime(){
	full = true;
	flight_cooldown -= Configs::flight_cooldowm_discharge;
}

void Shuttle::setLand(double x, double y){
	land_x = x;
	land_y = y;
}

void Shuttle::setBase(double x, double y){
	base_x = x;
	base_y = y;
}