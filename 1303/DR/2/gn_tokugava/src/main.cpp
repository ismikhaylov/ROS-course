#include "ros/ros.h"
#include "tokugava.h"

int main(int argc, char **argv)
{
  init(argc, argv, "gn_tokugava");

  GnTokugava general;
  general.waitForDoctor();
  general.listenToDoctor();

  return 0;
}