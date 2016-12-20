#ifndef DEFENCEUNIT_H
#define DEFENCEUNIT_H

#include "Unit.h"

class DefenceUnit: public Unit{
private:
	int 				defence_frame_count;
	vector<Texture*> 	defence_sprites;

  	void loadDefenceSprites(string& unit_type, string dir);

public:
	double				defence_cast;
  int           protect_id;
	double				defence_cooldown;
  bool          defence;
	DefenceUnit(NodeHandle& node, string unit_type, double x, double y, int frame_count, int defence_frame_count);
  	virtual Sprite 		getSprite();
  	virtual void		moveTo(double x, double y);
  	virtual void 		inputReaction(const geometry_msgs::Twist::ConstPtr& msg);
    virtual void 		nextFrame();
    virtual double    getRightBoard();
    virtual double    getBottomBoard();
  	bool 				isInDefenceZone(Point p);
  	void				doDefence();
};

#endif