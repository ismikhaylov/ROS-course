#ifndef CORE_H
#define CORE_H

#include <map>
#include <vector>
#include <string>
#include <memory>
#include <unordered_map>

#include "ros/ros.h"
#include "Configs.h"
#include "SFML/Graphics.hpp"

using namespace std;
using namespace ros;
using namespace sf;

namespace StarCraft {
	class FailedLoadTexture 
	{

	};

	class Core {
	public:
		static Core& getInstance();

		Texture* getTexture(const string& unit_type, const string& animation_type, int number_type);

		~Core();

	private:
		Core();
		Core(const Core&);
		Core& operator=(Core&);

		map<string, Texture> textures;
		string m_dir_texture;
	};
}

#endif