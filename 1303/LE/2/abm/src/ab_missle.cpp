#include "ros/ros.h"
#include "position_msg/Position.h"
#include <string>
#include <random>
#include <vector>
#include <algorithm>
#include <iostream>

class ABM {
  position_msg::Position pos;
public:
  ABM() {}
  ABM(float x, float y);

  bool canSave(float x, float y) const;
  void toString() const;
  float getX() const;
  float getY() const;
};

ABM::ABM(float x, float y) {
  pos.x = x;
  pos.y = y;
}

bool ABM::canSave(float x, float y) const {
  return pos.x == x && pos.y == y;
}

void ABM::toString() const {
  std::cout << "\n   ABM [" << pos.x << ", " << pos.y << "]\n";
}

float ABM::getX() const {
  return pos.x;
}

float ABM::getY() const {
  return pos.y;
}

struct Point {
  int x;
  int y;
  Point () : x(0), y(0) {}
  Point (int x, int y) : x(x), y(y) {}
  bool operator<(const Point &other);
};

bool Point::operator<(const Point &other) {
  if (y > other.y) {
    return true;
  } else if (y == other.y) {
    return x < other.x;
  }
  return false;
}

class Possibility {
  static std::default_random_engine generator;
  static std::uniform_int_distribution<int> distribution;
public:
  static bool possibility (int perCent);
};

std::default_random_engine Possibility::generator;
std::uniform_int_distribution<int> Possibility::distribution(0,99);

bool Possibility::possibility (int perCent) {
  int number = distribution(generator);
  return number < perCent;
}

const int ABMNum = 3;
ABM abms[ABMNum];

class Painter {
  float scaleX;
  float scaleY;
  Point misslePos;
  bool success;
  bool missleFall;
  const ABM* abms;
  int abmsNum;
  int fieldLength;
  int fieldHeight;

  float scale(const float* coordinates, int ABMNum, int real);

public:
  Painter(float x, float y, bool isSuccess, const ABM* curAbms, int ABMNum);
  void paint();

};

Painter::Painter(float x, float y, bool isSuccess, const ABM* curAbms, int ABMNum) {
  success = isSuccess;
  abms = curAbms;
  abmsNum = ABMNum;
  fieldLength = 50;
  fieldHeight = 20;
  // Сделать сдвиг в случае отрицательных координат.
  float minX = 0;
  float minY = 0;
  for (int i = 0; i < ABMNum; i++) {
    minX = abms[i].getX() < minX ? abms[i].getX() : minX;
  }
  for (int i = 0; i < ABMNum; i++) {
    minY = abms[i].getY() < minY ? abms[i].getY() : minY;
  }
  minX = x < minX ? x : minX;
  minY = y < minY ? y : minY;
  // Посчитать масштаб.
  float coordinatesX[ABMNum];
  float coordinatesY[ABMNum];
  for (int i = 0; i < ABMNum; i++) {
    coordinatesX[i] = abms[i].getX() + minX;
    coordinatesY[i] = abms[i].getY() + minY;
  }
  scaleX = scale(coordinatesX, ABMNum, fieldLength);
  scaleY = scale(coordinatesX, ABMNum, fieldHeight);
  std::cout << scaleX << "\n";
  std::cout << scaleY << "\n";
  missleFall = x <= (scaleX * fieldLength);

  Point points[ABMNum];
  for (int i = 0; i < ABMNum; i++) {
    points[i].x = int((abms[i].getX() - minX) / scaleX);
    points[i].y = int((abms[i].getY() - minY) / scaleY);
  }
  misslePos.x = int((x - minX) / scaleX);
  misslePos.y = int((y - minY) / scaleY);
}

float Painter::scale(const float* coordinates, int ABMNum, int real) {
  float max = 0;
  for (int i = 0; i < ABMNum; i++) {
    max = coordinates[i] > max ? coordinates[i] : max;
  }
  return max/real;
}

void Painter::paint() {
  int upHeight;
  int upLength;
  int length;
  int downHeight;
  int downLength;
  // Ракета улетела дальше самой дальней противоракетной установки
  if (!missleFall) {
    upHeight = fieldHeight / 2;
    upLength = fieldLength - 2;
    length = 2;
  } else {
    // Идеальная парабола
    if (misslePos.y == 0) {
      upLength = misslePos.x / 3;
      downLength = misslePos.x / 3;
      length = misslePos.x - upLength - downLength;
      upHeight = downLength / 2;
      downHeight = downLength / 2;
    } else {
      upLength = 2 * misslePos.x / 3;
      downLength = misslePos.x / 6;
      length = misslePos.x - upLength - downLength;
      upHeight = misslePos.y;
      upHeight += downLength / 2;
      downHeight = downLength / 2;
    }
  }
  // Посчитать количество следов
  int count = upHeight + downHeight;
  int lineCount = 1 + length / 5;
  count += lineCount;
  // Создать карту траектории ракеты
  std::vector<Point> positions;

  if (upLength > upHeight) {
    int prev = 0;
    int spaces = 0;
    if (upHeight > 0) {
      spaces = (upLength - upHeight)/upHeight;
    }
    positions.emplace_back();
    for (int i = 1; i < upHeight; i++) {
      positions.emplace_back(i*(spaces + 1), i);
      prev = i*(spaces +1);
    }
    spaces = (length - lineCount)/(lineCount + 1);
    for (int i = 0; i < lineCount; i++) {
       prev += spaces + 1;
       positions.emplace_back(prev, upHeight);
    }
    prev += spaces;
    spaces = 0;
    if (downHeight > 0) {
      spaces = (downLength - downHeight)/downHeight;
    }
    for (int i = 1; i <= downHeight; i++) {
      prev += spaces + 1;
      positions.emplace_back(prev, upHeight - i);
    }

  } else {
    // Иначе линия
    int spaces = 0;
    positions.emplace_back();
    if (upLength > 0)
      spaces = (upHeight - upLength)/upLength;
    for (int i = 1; i < upLength; i++) {
      positions.emplace_back(i, i*(spaces +1));
    }
  }
  
  
  // Непосредственно отрисовка
  std::sort(positions.begin(), positions.end());
  int curY = -fieldHeight - fieldHeight/10;
  int curX = 0;
  int index = 0;
  Point step = positions[0];
  Point last = positions[positions.size() - 1];
  while (curY != 1) {
    std::cout<< "|";
    curX = 0;
    while (curX < fieldLength) {
      while (curX < step.x && curX < fieldLength) {
        std::cout<< " ";
        curX++;
      }
      if (curY == -step.y) {
        std::cout << "*";
        index++;
        step = positions[index];
      }
      curX++;
    }
    std::cout << "\n";
    curY++;
  }
}

bool isAccidentalSuccess() {
  return Possibility::possibility(5);
}

bool isAccidentalFail() {
  return Possibility::possibility(20);
}

void MissleInAir(const position_msg::Position& msg) {
  bool expectedSuccess = false;
  for (int i = 0; i < ABMNum; i++) {
    expectedSuccess = expectedSuccess || abms[i].canSave(msg.x, msg.y);
  }
  std::cout <<"Missle fall position [" << msg.x << ", " << msg.y << "]\n";
  // Место падения угадано.
  if (expectedSuccess) {
    if (isAccidentalFail()) {
      std::cout <<"YOU ARE UNLUCKY. You set anti-ballistic missles right way. But accident occures and you FAIL!!!\n";
      Painter(msg.x, msg.y, false, abms, ABMNum).paint();
    } else {
      std::cout <<"SUCCESS. You set anti-ballistic missles right way.\n";
      Painter(msg.x, msg.y, true, abms, ABMNum).paint();
    }
  } else {
    if (isAccidentalSuccess()) {
      std::cout <<"YOU ARE LUCKY. You set anti-ballistic missles wrong way." << 
      " But accident occures with missle and you WIN!!!\n";
      Painter(msg.x, msg.y, true, abms, ABMNum).paint();
    } else {
      std::cout <<"FAIL. You set anti-ballistic missles wrong way.\n";
      Painter(msg.x, msg.y, false, abms, ABMNum).paint();
    }
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ab_missle");
  ros::NodeHandle handler;
  if (argc < ABMNum*2 + 1) {
    ROS_ERROR("It's necessary to set positions of anti-ballistic missles.");
    return 1;
  }
  std::cout <<"Anti-ballistic missles:\n";
  // Создаем противоракетные установки.
  for (int i = 0; i < ABMNum; i++) {
    abms[i] = ABM(std::stof(argv[i*2 + 1]), std::stof(argv[i*2 + 2]));
    abms[i].toString();
  }

  std::cout << "Wait a missle\n";
  ros::Subscriber subscriber = handler.subscribe("missle_pos" , 1000, MissleInAir);

  ros::spin();
  return 0;
}