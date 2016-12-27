#include "smart_bot.h"

int main( int argc, char** argv )
{
  init(argc, argv, "SmartBot");
  NodeHandle nodeHandle;

  SmartBot bot(nodeHandle, 7, 7);
  bot.findTheFriend();
  bot.goBack();

  return 0;
}