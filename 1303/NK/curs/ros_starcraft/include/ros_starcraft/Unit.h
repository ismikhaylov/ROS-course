#ifndef UNIT_H
#define UNIT_H

#include <vector>
#include <string>
#include "ros/ros.h"
#include "Configs.h"
#include "SFML/Graphics.hpp"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "Core.h"

using namespace ros;
using namespace sf;

struct Point{
  double x;
  double y;
  double w;

  Point(double x, double y, double w){
    this->x = x;
    this->y = y;
    this->w = w;
  }

  Point(){
    this->x = 0;
    this->y = 0;
    this->w = 0;
  }
};

class Unit{
private:
  int               frame_count;
	string 				    path_to_img;
	vector<Texture*> 	common_sprites;
	string 				    id_name;
	int               id;

  void loadSprites(string& unit_type, string dir);
  void applyPose(double x, double y, int dir);

protected:
	string          input_topic;
  Subscriber      input_sub;
  NodeHandle*     node;
  unsigned int    curr_sprite;

  virtual void 	inputReaction(const geometry_msgs::Twist::ConstPtr& msg);
  virtual void	moveTo(double x, double y);
  virtual void 	nextFrame();
  double 			getAngleTo(double x, double y);

public:
	static int             global_counter;
	static vector<string>* unit_ids;
	bool                   collisions[4];
  bool                   collision;
	bool                   dead;
  double                 x;
  double                 y;
  unsigned int           direction;

	Unit(NodeHandle& node, string unit_type, double x, double y, int frame_count);

  double 				getTopBoard();
  double 				getLeftBoard();
  double				getFrameHeight();
  double				getFrameWidth();
  virtual double 		getCenterX();
  virtual double 		getCenterY();
  virtual Sprite 		getSprite();
  virtual double 		getRightBoard();
  virtual double 		getBottomBoard();
  void              broadcastPose();

  bool  isInZone(double x, double y);
  bool 	isInVisibleZone(Point p);
  bool  isInFrame(double x, double y);
  void  lookTo(double x, double y);
  void 	goInDirection(double x, double y);
  void  deleteSelf();
};
#endif