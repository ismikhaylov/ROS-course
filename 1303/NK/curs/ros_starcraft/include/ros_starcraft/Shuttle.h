#ifndef SHUTTLE_H
#define SHUTTLE_H

#include "Unit.h"

class Shuttle: public Unit{
private:
	double 	land_x;
	double 	land_y;

	double 	base_x;
	double 	base_y;


public:
	double 	flight_cooldown;
	bool 	full;

	Shuttle(NodeHandle& node, string unit_type, double x, double y, int frame_count);
	bool isInLandingZone();
	bool isInBaseZone();
	void putOut();
	void goToBase();
	void goToLand();
	void wasteTime();
	void setLand(double x, double y);
	void setBase(double x, double y);

};
#endif