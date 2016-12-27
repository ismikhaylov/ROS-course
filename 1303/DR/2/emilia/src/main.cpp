#include "ros/ros.h"
#include "emilia.h"

int main(int argc, char **argv)
{
  init(argc, argv, "emilia");

  Emilia hotGirl;
  hotGirl.waitForDoctor();
  hotGirl.listenToDoctor();

  return 0;
}