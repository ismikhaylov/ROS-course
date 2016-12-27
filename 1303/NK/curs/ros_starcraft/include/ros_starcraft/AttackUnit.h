#ifndef ATTACKUNIT_H
#define ATTACKUNIT_H

#include "Unit.h"

class AttackUnit: public Unit{
private:
	int 				attack_frame_count;
	vector<Texture*> 	attack_sprites;
	bool				attack;
	Publisher 			target_pub;

  	void loadAttackSprites(string& unit_type, string dir);

public:
	AttackUnit(NodeHandle& node, string unit_type, double x, double y, int frame_count, int attack_frame_count);
  	virtual void 		inputReaction(const geometry_msgs::Twist::ConstPtr& msg);
  	virtual Sprite 		getSprite();
  	virtual void		 moveTo(double x, double y);
  	virtual double 		getRightBoard();
  	virtual double 		getBottomBoard();
    virtual void 		nextFrame();

  	bool 		isInAttackZone(Point p);
  	void 		doAttack(string target);

};
#endif