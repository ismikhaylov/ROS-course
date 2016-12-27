#ifndef ZERGSFRACTION_H
#define ZERGSFRACTION_H

#include "tf/transform_listener.h"
#include "Kerrigan.h"
#include "DefenceUnit.h"
#include "Avanpost.h"
#include "Configs.h"
#include <list>
#include <set>
#include <map>

class ZergsFraction{
private:
	Kerrigan* 			kerrigan;
	list<DefenceUnit*> 	zerglings;
	vector<Avanpost*> 	avanposts;
	map<string, Point> 	tampliers;
  	NodeHandle* 		node;
  	tf::TransformListener 	listener;

	void doKerriganLogic();
	void doKerriganDefenceLogic();
	void doZergLogic();
	void doZergDefenceLogic(DefenceUnit* z);
	void doAvanpostLogic();
	void updateTampliers();
	
public:
	bool	lost_avanpost;

	ZergsFraction(NodeHandle& node);
	void 			doLogic();
	void 			checkAvanpostControl();
	vector<Unit*> 	getAllUnits();
	vector<Unit*> 	getDinamicUnits();
	vector<Unit*> 	getStaticUnits();
};
#endif