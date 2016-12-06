#include <irrlicht.h>
#include <irrworld.h>
#include <string>
#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <ros/package.h>
#include <stack>
#include <queue>
#include <list>

using namespace irr;
using namespace core;
using namespace sensor_msgs;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;
using namespace std;
using namespace tf;
using namespace ros;
using namespace geometry_msgs;

/* LINKEDIT WITH IRRLICHT */

#ifdef _IRR_WINDOWS_
#pragma comment(lib, "Irrlicht.lib")
#pragma comment(linker, "/subsystem:windows /ENTRY:mainCRTStartup")
#endif

/* CUSTOMIZE LASER SCAN PROPERTIES */

#define MIN_LASER_SCAN_RANGE     0
#define MAX_LASER_SCAN_RANGE     4
#define DEFAULT_LASER_SCAN_RANGE MAX_LASER_SCAN_RANGE
#define MIN_LASER_SCAN_ANGLE     0
#define MAX_LASER_SCAN_ANGLE     2 * PI
#define NUM_LASER_SCAN_READINGS  360
#define LASER_SCAN_ANGLE_INC     (MAX_LASER_SCAN_ANGLE - MIN_LASER_SCAN_RANGE) / NUM_LASER_SCAN_READINGS
#define LASER_FREQUENCY          40

/* RE-DEFINITION OF IRRFLAG BITS */

enum
{
  IDFlag_IsPickable = 1 << 0
};

/* COMMON GLOBAL VARIABLES */

string current_dir;

// each next pose passed from controller will be queued here
// and then will be processed separatelly
// it will allow us to simulate slow motion
queue<PoseStamped> hero_pose_queu;

// Sauron's eye properties
float eye_target_x = 1;
float eye_target_y = 0.01;
float eye_target_z = 1;
float eye_radius = 0.5;

// Hero's properties
float hero_target_x = 7;
float hero_target_z = 4;
float helth = 100;

/* IRRLICHT ENVIRONMENT */

IAnimatedMeshSceneNode * hero = 0;
IMeshSceneNode * ps;

/* ROS ENVIRONMENT */

NodeHandle * nodeHandle = 0;
Publisher  * scan_publisher = 0;
Subscriber * hero_ps = 0;
Publisher  * goal_publisher = 0;
TransformBroadcaster * tf_broadcaster = 0;
PoseStamped goal_msg;

/* ROS CALLBACKS */

void heroPoseCallBack(const PoseStamped & msg)
{
  hero_pose_queu.push(msg);
}

void eyePoseCallBack(const Pose2D & msg)
{
  ps->setPosition(vector3df(msg.x, ps->getPosition().Y, msg.y));
}

/* COMMON FUNCTIONS AND PROCEDURES */

void initRosEnvironment()
{
  static NodeHandle n;
  nodeHandle = & n;

  static Subscriber hps = n.subscribe("/irrlicht/hero_pose", 500, heroPoseCallBack);
  hero_ps = & hps;
  static Subscriber sps = n.subscribe("/sauron/pose", 500, eyePoseCallBack);
  static Subscriber gps = n.subscribe("/gollum/pose", 500, gollumPoseCallBack);

  static Publisher gp = n.advertise<PoseStamped>("move_base_simple/goal", 500);
  goal_publisher = & gp;

  static Publisher sp = n.advertise<sensor_msgs::LaserScan>("scan", 500);
  scan_publisher = & sp;


  static TransformBroadcaster tb;
  tf_broadcaster = & tb;
}

void fillLaserScanRanges(LaserScan & scan)
{
  // create ray for raytracing
  line3d<f32> ray;
  ray.start = hero->getPosition();

  float scan_radius = MAX_LASER_SCAN_RANGE;

  // define raytracing result
  vector3df collisionPoint;
  triangle3df hitTriangle;

  // do 360 laser scans around 
  for(
    float i = 0, current_angle = MIN_LASER_SCAN_ANGLE; 
    current_angle <= MAX_LASER_SCAN_ANGLE && nodeHandle->ok(); 
    current_angle += LASER_SCAN_ANGLE_INC, i++
  )
  {
    float x = scan_radius * cos(current_angle) + ray.start.X;
    float z = scan_radius * sin(current_angle) + ray.start.Z;
    float y = ray.start.Y;

    ray.end = vector3df(x, y, z);

    ISceneNode * obstacle = collision_manager->getSceneNodeAndCollisionPointFromRay(
      ray,
      collisionPoint, 
      hitTriangle,
      IDFlag_IsPickable,
      0
    );

    float distanceToObstacle = DEFAULT_LASER_SCAN_RANGE;
    if(obstacle != 0) 
      distanceToObstacle = hero->getPosition().getDistanceFrom(collisionPoint);

    scan.ranges[i] = distanceToObstacle;
    scan.intensities[i] = distanceToObstacle + 5;
  }
}

void publishLaserScan()
{
  // send sensor pose over tf (relative to base)
  tf_broadcaster->sendTransform(
    tf::StampedTransform(
      tf::Transform(
        tf::Quaternion(0, 0, 0, 1),
        tf::Vector3(0, 0, 0)
      ),
      Time::now(),
      "base_link",
      "laser_frame"
    )
  );

  // populate the LaserScan message
  sensor_msgs::LaserScan scan;
  scan.header.stamp = Time::now();
  scan.header.frame_id = "laser_frame";
  scan.angle_min = MIN_LASER_SCAN_ANGLE;
  scan.angle_max = MAX_LASER_SCAN_ANGLE;
  scan.angle_increment = LASER_SCAN_ANGLE_INC;
  scan.time_increment = (1 / LASER_FREQUENCY) / (NUM_LASER_SCAN_READINGS);
  scan.range_min = MIN_LASER_SCAN_ANGLE;
  scan.range_max = MAX_LASER_SCAN_ANGLE;
  scan.ranges.resize(NUM_LASER_SCAN_READINGS);
  scan.intensities.resize(NUM_LASER_SCAN_READINGS);

  fillLaserScanRanges(scan);

  scan_publisher->publish(scan);
}

void updateHeroPose()
{
  // check if Hero went to the last target pose or not
  if (fabs(hero->getPosition().X - next_x) < 0.1 && fabs(hero->getPosition().Z - next_z) < 0.1)
  {
    // make the Hero standing
    if (running)
    {
      hero->setMD2Animation(EMAT_STAND);
      running = false;
    }

    // initialize movement to the next pose (if it is exist)
    if (!hero_pose_queu.empty())
    {
      dx = 0.1;
      dz = 0.1;

      PoseStamped msg = hero_pose_queu.front();
      hero_pose_queu.pop();

      next_x = msg.pose.position.x;
      next_z = msg.pose.position.y;
      next_th = msg.pose.orientation.z;

      dx *= hero->getPosition().X < next_x ? 1 : -1;
      dz *= hero->getPosition().Z < next_z ? 1 : -1;
    }
  }
  else
  {
    // make the Hero running
    if (!running)
    {
      hero->setMD2Animation(EMAT_RUN);
      running = true;
    }

    // get position and rotation of the Hero
    vector3df position = hero->getPosition();
    vector3df rotation = hero->getRotation();

    rotation.Y = -next_th * RADTODEG;

    if (fabs(hero->getPosition().X - next_x) >= 0.1)
      position.X += dx;

    if (fabs(hero->getPosition().Z - next_z) >= 0.1)
      position.Z += dz;

    // set back updated properties
    hero->setPosition(position);
    hero->setRotation(rotation);
  }
}

int main(int argc, char ** argv)
{
  // init all things
  init(argc, argv, "irrlicht");
  initIrrlichtEnvironment();
  initRosEnvironment();

  // the next proc will load 3D world with all predefined nodes
  // it will also expand all interfaces to access exterlan variables
  // defined in world.irr and irrlicht lib
  loadIrrlichtWorld("world.irr");

  // init goal msg
  goal_msg.pose.position.x = hero_target_x;
  goal_msg.pose.position.y = hero_target_z;
  goal_msg.header.frame_id = "/map";
  goal_msg.pose.orientation.w = 1;


  while (device->run() && nodeHandle->ok())
  {
    // update external info
    spinOnce();

    // send all required data to ROS
    publishLaserScan();

    // draw the scene
    driver->beginScene(true, true, SColor(255, 100, 101, 140));
    scene_manager->drawAll();
    guienv->drawAll();
    driver->endScene();

    // update scene for the next step
    updateHeroPose();

    // publish goal for nav stack
    goal_publisher->publish(goal_msg);

    // print status just for debug reason
    cout << "helth: (" << helth << ") pose: (" << hero->getPosition().X << ' ' << hero->getPosition().Z << ")" << endl;

    // check if we did our goal
    if(fabs(hero->getPosition().X - hero_target_x) < 0.1 && fabs(hero->getPosition().Z - hero_target_z) < 0.1)
    {
      cout << "\n\n* * * WE ARE ALL SET * * *\n\n" << endl;
      cin.get();
      return 0;
    }

    // remember about the EYE
    if(fabs(hero->getPosition().X - eye_target_x) < eye_radius && fabs(hero->getPosition().Z - eye_target_z) < eye_radius)
    {
      cout << "Oh my Grandmather.. Sauron has looked at me" << endl;
      helth -= 10;
    }
  }

  // it is a smart ptr which is not really smart as well
  // so we must explicitly drop the reference
  device->drop();

  return 0;
}