#include "Configs.h"

string Configs::root_path = "/home/ckyhc/ros/labs/workspace";

double Configs::step_delta = 0;

double Configs::window_h = 0;
double Configs::window_w = 0;

double Configs::unit_speed = 0;
double Configs::frame_count = 0;
double Configs::visible_radius = 0;

double Configs::defence_radius = 0;
double Configs::shield_cast_time = 0;
double Configs::shield_cooldowm_time = 0;
double Configs::shield_discarge = 0;

double Configs::attack_radius = 0;

double Configs::avanposts_count = 0;
double Configs::zerg_create_time = 0;
double Configs::zerg_create_discarge = 0;
double Configs::avanpost_control_radius = 0;

double Configs::transports_count = 0;
double Configs::transport_capability = 0;
double Configs::flight_cooldown_time = 0;
double Configs::flight_cooldowm_discharge = 0;

double Configs::tampliers_count_limit = 0;

vector<string> Configs::split(string str){
	vector<string> output;

	size_t delim_pos = str.find('=');

	if(delim_pos == string::npos){
		cout << "Synax error at string \"" << str << "\"" << "at file \"config.cnf\".\n";
		return output;
	}

	string variable_name = str.substr(0, delim_pos);
	string variable_val = str.substr(delim_pos + 1, str.length());

	if(variable_name.length() == 0){
		cout << "Synax error at string \"" << str << "\"" << "at file \"config.cnf\".\n";
		return output;
	}
	if(variable_val.length() == 0){
		cout << "Synax error at string \"" << str << "\"" << "at file \"config.cnf\".\n";
		return output;
	}

	output.push_back(variable_name);
	output.push_back(variable_val);

	return output;
}

void Configs::parse(){
	string path = root_path + "/src/ros_starcraft/config/config.cnf";
	fstream conf_file;
	conf_file.open(path.c_str());

	if(!conf_file){
		cout << "Config file \"config.cnf\" was not found.\n";
		return;
	}

	string input;

	while(!conf_file.eof()){
		conf_file >> input;

		if(input[0] != '#' && input[0] != '\n'){
			vector<string> var_set = split(input);
			string var_name = var_set[0];
			double var_val = strtod(var_set[1].c_str(), 0);

			if (var_name.compare("step_delta") == 0) step_delta = var_val;
			
			else if (var_name.compare("window_h") == 0) window_h = var_val;
			else if (var_name.compare("window_w") == 0) window_w = var_val;
			
			else if (var_name.compare("unit_speed") == 0) unit_speed = var_val;
			else if (var_name.compare("frame_count") == 0) frame_count = var_val;
			else if (var_name.compare("visible_radius") == 0) visible_radius = var_val;

			else if (var_name.compare("defence_radius") == 0) defence_radius = var_val;
			else if (var_name.compare("shield_cast_time") == 0) shield_cast_time = var_val;
			else if (var_name.compare("shield_cooldowm_time") == 0) shield_cooldowm_time = var_val;
			else if (var_name.compare("shield_discarge") == 0) shield_discarge = var_val;

			else if (var_name.compare("attack_radius") == 0) attack_radius = var_val;

			else if (var_name.compare("avanposts_count") == 0) avanposts_count = var_val;
			else if (var_name.compare("zerg_create_time") == 0) zerg_create_time = var_val;
			else if (var_name.compare("zerg_create_discarge") == 0) zerg_create_discarge = var_val;
			else if (var_name.compare("avanpost_control_radius") == 0) avanpost_control_radius = var_val;

			else if (var_name.compare("transports_count") == 0) transports_count = var_val;
			else if (var_name.compare("transport_capability") == 0) transport_capability = var_val;
			else if (var_name.compare("flight_cooldown_time") == 0) flight_cooldown_time = var_val;
			else if (var_name.compare("flight_cooldowm_discharge") == 0) flight_cooldowm_discharge = var_val;

			else if (var_name.compare("tampliers_count_limit") == 0) tampliers_count_limit = var_val;
		}
	}
}