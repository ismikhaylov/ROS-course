#include "ZergsFraction.h"

ZergsFraction::ZergsFraction(NodeHandle& node){
	this->node = &node;
	kerrigan = new Kerrigan(node, "kerrigan", 0.0, 200.0, 8, 8);
	kerrigan->direction = up;
	for(int i = 0; i < Configs::avanposts_count; i++){
		Avanpost* avan = new Avanpost(node, "incubator", 100.0*i + 100.0, 0.0, 3);
		avan->direction = up;
		avanposts.push_back(avan);
	}
	lost_avanpost = false;
}

void ZergsFraction::doKerriganLogic(){
	pair<string, Point> agro_target;
	pair<string, Point> attack_target;

	for(pair<string, Point> t: tampliers){

		if(kerrigan->isInAttackZone(t.second)){
			attack_target = t;

		} else if(kerrigan->isInVisibleZone(t.second)){
			agro_target = t;
		}
	}

	if(!attack_target.first.empty()){
		kerrigan->lookTo(attack_target.second.x, attack_target.second.y);
		kerrigan->doAttack(attack_target.first);
	} else if(!agro_target.first.empty()){
		kerrigan->goInDirection(agro_target.second.x, agro_target.second.y);
	} else {
		doKerriganDefenceLogic();
	}
}

void ZergsFraction::doKerriganDefenceLogic(){
	Avanpost* avan = avanposts[kerrigan->protect_id];
	kerrigan->goInDirection(avan->getLeftBoard(), avan->getBottomBoard() + 100);

	if(kerrigan->isInZone(avan->getLeftBoard(), avan->getBottomBoard()))
		kerrigan->protect_id = rand() % (int)Configs::avanposts_count;
}

void ZergsFraction::doZergDefenceLogic(DefenceUnit* z){ 
	Avanpost* avan = avanposts[z->protect_id]; 
	if (!z->defence) { 
		z->goInDirection(avan->getRightBoard(), avan->getBottomBoard() + 200); 

		if(z->isInZone(avan->getLeftBoard(), avan->getBottomBoard())) 
			z->protect_id = rand() % (int)Configs::avanposts_count; 
	} else {
		z->defence = false;
		for(pair<string, Point> t: tampliers){
			if(z->isInVisibleZone(t.second)){
				z->defence = true;
				break;
			}
		}
	}
}

void ZergsFraction::doAvanpostLogic(){
	for(Avanpost* a: avanposts){
		if(a->zerg_create_progress <= 0){
			double default_y = a->getBottomBoard();
			double default_x = a->getRightBoard();
			zerglings.push_back(new DefenceUnit(*node, "zergling", default_x, default_y, 12, 1));
			a->zerg_create_progress = Configs::zerg_create_time;
		}else{
			a->zerg_create_progress -= Configs::zerg_create_discarge;
		}
		a->broadcastPose();
	}
	checkAvanpostControl();
}

void ZergsFraction::updateTampliers(){
	tampliers.clear();
	for(string id: *Unit::unit_ids){
		std::size_t pos = id.find("tamplier");
		if(string::npos != pos){
			tf::StampedTransform tamplier_pos;
			try{
      			listener.lookupTransform("world", id, ros::Time(0), tamplier_pos);
    		} catch (tf::TransformException &ex) {
      			ROS_ERROR("%s",ex.what());
    		}
    		Point tamplier(tamplier_pos.getOrigin().x(),   tamplier_pos.getOrigin().y(),
                           tamplier_pos.getOrigin().z());
    		tampliers.insert(pair<string, Point>(id, tamplier));
		}
	}	
}

void ZergsFraction::doZergLogic(){
	for(list<DefenceUnit*>::iterator it = zerglings.begin(); it != zerglings.end();){
		if((*it)->dead){
			delete *it;
			it = zerglings.erase(it);
		} else if((*it)->direction == death){
			(*it)->nextFrame();
			++it;
		} else{
			doZergDefenceLogic(*it);
			++it;
		}
	}
}

void ZergsFraction::doLogic(){
	updateTampliers();
	doZergLogic();
	doKerriganLogic();
	doAvanpostLogic();
}

void ZergsFraction::checkAvanpostControl(){
	Point k_p(kerrigan->x, kerrigan->y, kerrigan->getFrameWidth());
	bool next;
	bool exit = false;
	lost_avanpost = false;

	for(int i = 0; i < avanposts.size() && !exit; i++){
		next = false;

		for(pair<string, Point> t: tampliers){
			if(next || exit) break;

			if(avanposts[i]->isInControlZone(t.second)){
				avanposts[i]->underZerg = avanposts[i]->isInControlZone(k_p);

				if(avanposts[i]->underZerg) next = true;
				else{

					for(DefenceUnit* z: zerglings){
						if(z->direction != death){
							if(next || exit) break;

							Point z_p(z->x, z->y, z->getFrameWidth());

							avanposts[i]->underZerg = avanposts[i]->isInControlZone(z_p);
							if(avanposts[i]->underZerg){
								next = true;
							}
							else {
								lost_avanpost = true;
								exit = true;
							}
						}else{
							lost_avanpost = true;
						}
					}
					if(zerglings.size() == 0){
						lost_avanpost = true;
					}
				}

			}
		}
	}
}

vector<Unit*> ZergsFraction::getAllUnits(){
	std::set<Unit*> all;
	all.insert(kerrigan);
	all.insert(zerglings.begin(), zerglings.end());
	all.insert(avanposts.begin(), avanposts.end());
	std::vector<Unit*> result(all.begin(), all.end());
	return result;
}

vector<Unit*> ZergsFraction::getDinamicUnits(){
	std::set<Unit*> all;
	all.insert(kerrigan);
	all.insert(zerglings.begin(), zerglings.end());
	std::vector<Unit*> result(all.begin(), all.end());
	return result;
}

vector<Unit*> ZergsFraction::getStaticUnits(){
	std::set<Unit*> all;
	all.insert(avanposts.begin(), avanposts.end());
	std::vector<Unit*> result(all.begin(), all.end());
	return result;
}