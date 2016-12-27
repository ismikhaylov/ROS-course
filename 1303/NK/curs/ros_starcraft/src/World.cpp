#include "World.h"

World::World(NodeHandle& node){

	zf = new ZergsFraction(node);
	pf = new ProtossFraction(node);
	pause = false;

	window = new RenderWindow(sf::VideoMode(Configs::window_w, Configs::window_h), "SFML works!");
	string type = "ground2";
	loadSprites(type, "up", 26);
}

void World::loadSprites(string& unit_type, string dir, int frame_count){
	for(int i = 0; i < frame_count; i++){
		Texture* t = StarCraft::Core::getInstance().getTexture(unit_type, dir, i);
		ground_sprites.push_back(t);
	}
}

void World::drawGround(){
	int size = Configs::window_w/30;
	int count = ground_sprites.size();
	int w = ground_sprites[0]->getSize().x;

	for(int i = 0; i < size; i++){
		for (int j = 0; j < size; j++){

			int id = (i*size + j) % count;
			Sprite s;
			s.setTexture(*ground_sprites[id]);
			s.setPosition(i*w, j*w);

			window->draw(s);
		}
	}
}

void World::start(){
	ros::Rate r(10);
	while (window->isOpen() && ros::ok())
    {
        sf::Event event;
        while (window->pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window->close();
        }

        if(!pause){
	        window->clear();

			checkCollisions();

	        zf->doLogic();
	        pf->doLogic();

			updateWindow();
	        
	        window->display();
			checkWin();
		}

        r.sleep();
        ros::spinOnce();       
    }
}

void World::updateWindow(){	
	drawGround();

	vector<Unit*> static_units = zf->getStaticUnits();
    for(int i = 0; i < static_units.size(); i++){
    	window->draw(static_units[i]->getSprite());
    }

	vector<Unit*> ground_units = getDinamicUnits();
    for(int i = 0; i < ground_units.size(); i++){
    	if(ground_units[i]->direction == death){
	    	window->draw(ground_units[i]->getSprite());
	    }
    }

    for(int i = 0; i < ground_units.size(); i++){
    	if(ground_units[i]->direction != death){
	    	window->draw(ground_units[i]->getSprite());
	    }
    }

	vector<Unit*> air_protoss = pf->getShuttleUnits();
    for(int i = 0; i < air_protoss.size(); i++){
    	window->draw(air_protoss[i]->getSprite());
    }
}

void World::checkCollisions(){
	vector<Unit*> din_units = getDinamicUnits();
	vector<Unit*> stat_units = getStaticUnits();

	for(int i = 0; i < din_units.size(); i++){

		din_units[i]->collisions[right_] = false;
		din_units[i]->collisions[left_] = false;
		din_units[i]->collisions[up] = false;
		din_units[i]->collisions[down] = false;

		for(int j = 0; j < din_units.size(); j++){
			if (j != i) checkCollision(din_units[i], din_units[j]);
		}
		for(int j = 0; j < stat_units.size(); j++){
			checkCollision(din_units[i], stat_units[j]);
		}
	}
}

void World::checkCollision(Unit* unit, Unit* target){
	if(unit->direction != death && target->direction != death){
		double delta = 5;

		double dm = unit->getFrameWidth()/2;
		double x_right = unit->getRightBoard();
		double x_left = unit->getLeftBoard();
		double y_top = unit->getTopBoard();
		double y_bottom = unit->getBottomBoard();

		double y_m = y_top + dm;
		double x_m = x_left + dm;

		if(target->isInFrame(x_right + delta, y_top) || target->isInFrame(x_right + delta, y_bottom) || target->isInFrame(x_right + delta, y_m)){
			unit->collisions[right_] = true;
		}

		if(target->isInFrame(x_left - delta, y_top) || target->isInFrame(x_left - delta, y_bottom) || target->isInFrame(x_left - delta, y_m)){
			unit->collisions[left_] = true;
		}

		if(target->isInFrame(x_left, y_top - delta) || target->isInFrame(x_right, y_top - delta) || target->isInFrame(x_m, y_top - delta)){
			unit->collisions[up] = true;
		}

		if(target->isInFrame(x_left, y_bottom + delta) || target->isInFrame(x_right, y_bottom + delta) || target->isInFrame(x_m, y_bottom + delta)){
			unit->collisions[down] = true;
		}
	}
}

vector<Unit*> World::getStaticUnits(){
	return zf->getStaticUnits();
}

vector<Unit*> World::getDinamicUnits(){
	vector<Unit*> zergs = zf->getDinamicUnits();
	vector<Unit*> protoss = pf->getTampliersUnits();

	std::set<Unit*> all;
	all.insert(zergs.begin(), zergs.end());
	all.insert(protoss.begin(), protoss.end());
	std::vector<Unit*> result(all.begin(), all.end());
	return result;
}

void World::checkWin(){
	if(zf->lost_avanpost){
		pause = true;
		cout << "Protosses win!" << endl;
	}
	if(pf->loose){
		pause = true;
		cout << "Zergs win!" << endl;
	}
}