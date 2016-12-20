#ifndef WORLD_H
#define WORLD_H

#include "ZergsFraction.h"
#include "ProtossFraction.h"

class World{
public:
	vector<Texture*> 	ground_sprites;

	World(NodeHandle& node);
	RenderWindow* window;

	ZergsFraction* zf;
	ProtossFraction* pf;
	bool pause;

    void loadSprites(string& unit_type, string dir, int frame_count);
    void drawGround();

	void start();
	void updateWindow();
	void checkCollisions();
	void checkCollision(Unit*, Unit*);
	vector<Unit*> getStaticUnits();
	vector<Unit*> getDinamicUnits();
	void checkWin();

};

#endif