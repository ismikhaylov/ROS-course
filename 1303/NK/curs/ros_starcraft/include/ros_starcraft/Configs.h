#ifndef CONFIGS_H
#define CONFIGS_H

#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <cstring>
#include <cstdlib>

using  namespace std;

enum directions {up = 0, down, left_, right_, up_left, up_right, down_left, down_right, death};

class Configs{
private:
	string 			config_filename;
	vector<string> 	split(string str);

public:
	static string root_path;

	static double step_delta;

	static double window_h;
	static double window_w;

	static double unit_speed;
	static double frame_count;
	static double visible_radius;

	static double defence_radius;
	static double shield_cast_time;
	static double shield_cooldowm_time;
	static double shield_discarge;

	static double attack_radius;

	static double avanposts_count;
	static double zerg_create_time;
	static double zerg_create_discarge;
	static double avanpost_control_radius;

	static double transports_count;
	static double transport_capability;
	static double flight_cooldown_time;
	static double flight_cooldowm_discharge;

	static double tampliers_count_limit;

	void parse();
};
#endif