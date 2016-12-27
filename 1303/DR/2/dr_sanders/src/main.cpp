#include "ros/ros.h"
#include "sanders.h"

int main(int argc, char **argv)
{
  init(argc, argv, "dr_sanders");

  DrSanders doctor;
  bool heIsReady = doctor.getReady();

  doctor.waitForColleagues();
  
  if(heIsReady)
  	doctor.tellReport();
  else 
  	doctor.tellStatus();

  return 0;
}