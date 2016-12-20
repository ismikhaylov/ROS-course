#include "Core.h"

StarCraft::Core::Core() 
{
	m_dir_texture = Configs::root_path + "/src/ros_starcraft/img/";
}

StarCraft::Core::~Core()
{

}

StarCraft::Core& StarCraft::Core::getInstance() 
{
	static Core instance;
	return instance;
}

Texture* StarCraft::Core::getTexture(const string& unit_type, const string& animation_type, int number_type)
{
	string path = m_dir_texture + unit_type + "/" + animation_type + to_string(number_type) + ".png";

	Texture *t;
    t = &textures[path];
    if( t->getSize().x == 0){
        t->loadFromFile(path);
    }

    return t;
}
