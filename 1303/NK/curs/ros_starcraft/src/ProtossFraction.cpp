#include "ProtossFraction.h"

ProtossFraction::ProtossFraction(NodeHandle& node){
  	loose = false;
	this->node = &node;
	curr_count = -1;
	for(int i = 0; i < Configs::transports_count; i++){
		Shuttle* sh = new Shuttle(node, "shuttle", 250.0*i, Configs::window_h + 100, 1);
		sh->setLand(150.0*i + 100.0,  Configs::window_h/2);
		shuttles.push_back(sh);
	}
 }

void ProtossFraction::doTampliersLogic(){	\
	updateZergs();\
	for(vector<AttackUnit*>::iterator it = tampliers.begin(); it != tampliers.end();){
		if((*it)->dead){\
			delete *it;
			it = tampliers.erase(it);
		} else{
            pair<string, Point> agro_target;
            pair<string, Point> attack_target;

            for(pair<string, Point> z: zergs){
                if((*it)->isInAttackZone(z.second)){
                    attack_target = z;
                } else if((*it)->isInVisibleZone(z.second)){
                    agro_target = z;
                }
            }

            if(!attack_target.first.empty()){
                (*it)->lookTo(attack_target.second.x, attack_target.second.y);
                (*it)->doAttack(attack_target.first);
            } else if(!agro_target.first.empty()){
                (*it)->goInDirection(agro_target.second.x, agro_target.second.y);
            } else {
                doTampliersRushLogic(*it);
            }
            it++;
        }
	}
}
void ProtossFraction::doTampliersRushLogic(AttackUnit* t){
	int id = rand() % (int)Configs::avanposts_count;
    Point avan = avanposts[id];
    t->goInDirection(avan.x, avan.y);
}

void ProtossFraction::doShuttlesLogic(){
	for(Shuttle* s: shuttles){
		if((s->flight_cooldown > 0 && s->full) || s->isInBaseZone()){
        	s->wasteTime();
        }
        if(s->flight_cooldown <= 0 && s->full && curr_count < Configs::tampliers_count_limit){
        	s->goToLand();
        }
        if(s->isInLandingZone() && s->full && curr_count < Configs::tampliers_count_limit){
        	s->putOut();
        	{
        		int startId = tampliers.size();
        		for(int i = 0; i < Configs::transport_capability; i++){
        			AttackUnit* t = new AttackUnit(*node, "tamplier", 0.0, 0.0, 8, 5);
        			tampliers.push_back(t);
        		}
        		if(curr_count == -1)
        			curr_count = 0;

        		curr_count += Configs::transport_capability;

        		int area_side = 0;
        		double area_x = s->getCenterX();
        		double area_y = s->getCenterY();

        		for(area_side = 0; area_side*area_side <Configs::transport_capability; area_side++);

        		for(int i = 0; i < area_side/2; i++){
        			area_x -= tampliers[0]->getFrameWidth();
        			area_y -= tampliers[0]->getFrameHeight();
        		}
        		
        		if(area_side%2 != 0){
        			area_x -= tampliers[0]->getFrameWidth()/2;
        			area_y -= tampliers[0]->getFrameHeight()/2;
        		}

        		int i = 0;
        		int j = 0;
        		while(i < area_side){
        			j = 0;
        			while(i*area_side + j < Configs::transport_capability && j < area_side){
        				int id = startId + i*area_side + j;
        				tampliers[id]->x = area_x + j*tampliers[id]->getFrameWidth();
        				tampliers[id]->y = area_y + i*tampliers[id]->getFrameHeight();
        				j++;
        			}
        			i++;
        		}

    		}
        }
        if(!s->full){
        	s->goToBase();
        }
    }
}

void ProtossFraction::updateZergs(){
	zergs.clear();
	avanposts.clear();

	for(string id: *Unit::unit_ids){
		std::size_t pos = id.find("zerg");
		if(string::npos != pos){
			tf::StampedTransform zerg_pos;
			try{
      			listener.lookupTransform("world", id, ros::Time(0), zerg_pos);
    		} catch (tf::TransformException &ex) {
      			ROS_ERROR("%s",ex.what());
    		}
    		Point zerg(zerg_pos.getOrigin().x(),   zerg_pos.getOrigin().y(),
                       zerg_pos.getOrigin().z());
    		zergs.insert(pair<string, Point>(id, zerg));
		}

		pos = id.find("incubator");
		if(string::npos != pos){
			tf::StampedTransform avanpost_pos;
			try{
      			listener.lookupTransform("world", id, ros::Time(0), avanpost_pos);
    		} catch (tf::TransformException &ex) {
      			ROS_ERROR("%s",ex.what());
    		}
    		Point avanpost(avanpost_pos.getOrigin().x(), avanpost_pos.getOrigin().y(),
                avanpost_pos.getOrigin().z());
    		avanposts.push_back(avanpost);
		}
	}	
}
	
	
void ProtossFraction::doLogic(){
	doTampliersLogic();
	doShuttlesLogic();
	loose = tampliers.size() == 0 && curr_count >= Configs::tampliers_count_limit;
}

vector<Unit*> ProtossFraction::getAllUnits(){
	std::set<Unit*> all;
	all.insert(shuttles.begin(), shuttles.end());
	all.insert(tampliers.begin(), tampliers.end());
	std::vector<Unit*> result(all.begin(), all.end());
	return result;
}

vector<Unit*> ProtossFraction::getTampliersUnits(){
	std::vector<Unit*> result(tampliers.begin(), tampliers.end());
	return result;
}

vector<Unit*> ProtossFraction::getShuttleUnits(){
	std::vector<Unit*> result(shuttles.begin(), shuttles.end());
	return result;
}