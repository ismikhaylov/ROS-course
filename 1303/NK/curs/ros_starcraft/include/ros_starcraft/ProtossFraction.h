#ifndef PROTOSSFRACTION_H
#define PROTOSSFRACTION_H

#include "tf/transform_listener.h"
#include "AttackUnit.h"
#include "Shuttle.h"
#include "Configs.h"
#include <list>
#include <set>
#include <map>

class ProtossFraction{
private:
	vector<AttackUnit*> 	tampliers;
	vector<Shuttle*> 		shuttles;
	map<string, Point> 		zergs;
	vector<Point> 			avanposts;
  	NodeHandle* 			node;
  	tf::TransformListener 	listener;
  	int						curr_count;

	void doTampliersLogic();
	void doTampliersRushLogic(AttackUnit* t);
	void doShuttlesLogic();
	void updateZergs();
	
public:
  	bool			loose;

	ProtossFraction(NodeHandle& node);
	void 			doLogic();
	vector<Unit*> 	getAllUnits();
	vector<Unit*> 	getTampliersUnits();
	vector<Unit*> 	getShuttleUnits();
};
#endif

