#ifndef AVANPOST_H
#define AVANPOST_H

#include "Unit.h"

class Avanpost: public Unit{
public:
	Avanpost(NodeHandle& node, string unit_type, double x, double y, int frame_count);
	double 	zerg_create_progress;
	bool 	underZerg;
	bool 	isInControlZone(Point p);
	void	progress();
	void	startCreation();
};
#endif