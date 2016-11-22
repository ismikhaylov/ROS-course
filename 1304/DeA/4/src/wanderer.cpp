#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include "wandered_robot.h"

void generateWorldFile(const std::string& dir, const std::string& name, const std::string& image,
                       float robotX, float robotY, float goalX, float goalY)
{
    std::ofstream output(dir + "/" + name + ".world");
    output << "define block model\n"
           << "(\n"
           << "  size [0.500 0.500 0.500]\n"
           << "  gui_nose 0\n"
           << ")\n\n"
           << "define topurg ranger\n"
           << "(\n"
           << "  sensor (\n"
           << "    range [ 0.0  30.0 ]\n"
           << "    fov 180.0\n"
           << "    samples 200\n"
           << "  )\n"
           << "  # generic model properties\n"
           << "  color \"black\"\n"
           << "  size [ 0.050 0.050 0.100 ]\n"
           << ")\n\n"
           << "define goal_block position\n"
           << "(\n"
           << "  size [0.350 0.350 0.250]\n"
           << "  origin [0.000 0.000 0.000 0.000]\n"
           << "  gui_nose 0\n"
           << ")\n\n"
           << "define erratic position\n"
           << "(\n"
           << "  size [0.350 0.350 0.250]\n"
           << "  origin [-0.050 0.000 0.000 0.000]\n"
           << "  gui_nose 1\n"
           << "  drive \"diff\"\n"
           << "  topurg(pose [ 0.050 0.000 0.000 0.000 ])\n"
           << ")\n\n"
           << "define floorplan model\n"
           << "(\n"
           << "  # sombre, sensible, artistic\n"
           << "  color \"gray30\"\n"
           << "  # most maps will need a bounding box\n"
           << "  boundary 1\n"
           << "  gui_nose 0\n"
           << "  gui_grid 0\n"
           << "  gui_outline 0\n"
           << "  gripper_return 0\n"
           << "  fiducial_return 0\n"
           << "  laser_return 1\n"
           << ")\n\n"
           << "# set the resolution of the underlying raytrace model in meters\n"
           << "resolution 0.02\n\n"
           << "interval_sim 100  # simulation timestep in milliseconds\n"
           << "threads 2\n\n"
           << "window\n"
           << "(\n"
           << "  size [ 800 400 ]\n"
           << "  center [ 0.000 0.000 ]\n"
           << "  rotate [ 0.000 0.000 ]\n"
           << "  scale 16.000\n"
           << ")\n\n"
           << "# load an environment bitmap\n"
           << "floorplan\n"
           << "(\n"
           << "  name \"world\"\n"
           << "  bitmap \"" << image << "\"\n"
           << "  size [20.000 20.000 1.0]\n"
           << "  #pose [-25.0 25.0 0.000 0.000 ]\n"
           << ")\n\n"
           << "# throw in a robot\n"
           << "erratic( pose [ " << std::to_string(robotX) << " " << std::to_string(robotY)
           << " 0.000 0.000 ] name \"robot_0\" color \"blue\")\n\n"
           << "# throw goal point\n"
           << "goal_block(pose [ " << std::to_string(goalX) << " " << std::to_string(goalY)
           << " 0.000 0.000 ] name \"robot_1\" color \"red\")";
    output.close();
}

void generateSimulatorLaunchFile(const std::string& dir, const std::string& name, const std::string& worldFile)
{
    std::ofstream output(dir + "/" + name + ".launch");
    output << "<?xml version=\"1.0\"?>\n"
           << "<launch>\n"
           << "  <node pkg=\"stage_ros\" type=\"stageros\" name=\"simulator\""
           //<< " launch-prefix=\"xterm -e\""
           <<" args=\"$(find ros_lab4)/world/" << worldFile << "\" />\n"
           << "</launch>\n";
    output.close();

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wanderer");
    ros::NodeHandle nodeHandle;

    if (argc != 7)
    {
        ROS_ERROR("Run error: Need 7 arguments");
        return 1;
    }

    float robotX = atof(argv[1]);
    float robotY = atof(argv[2]);
    float goalX = atof(argv[3]);
    float goalY = atof(argv[4]);
    std::string worldName = argv[5];
    std::string worldImageName = argv[6];
    std::string projectDir = "/home/saydos/Desktop/ros/catkin_ws/src/ros_lab4";

    ROS_INFO("Generate run files");
    generateWorldFile(projectDir + "/world", worldName, worldImageName, robotX, robotY, goalX, goalY);
    generateSimulatorLaunchFile(projectDir + "/launch", worldName + "_map", worldName + ".world");
    std::string bashCommand = std::string("gnome-terminal --window -e \"/bin/bash -c \'cd $catkin_workspace;")
        + " roslaunch ros_lab4 " + worldName + "_map.launch; rosrun exec /bin/bash -i\'\"";
    ROS_INFO("Run simulator");
    system(bashCommand.c_str());
    sleep(3.0);
    ROS_INFO("Start wanderer");

    WanderedRobot robot(nodeHandle, robotX, robotY, goalX, goalY);
    robot.moveToGoal();

    return 0;
}
