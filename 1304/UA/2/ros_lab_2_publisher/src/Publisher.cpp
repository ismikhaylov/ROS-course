#include <ros/ros.h>
#include <chrono>
#include "ros_lab_2_msg/CryptoMessage.h"

void encryptMessage(std::string& message, uint8_t offset)
{
    for (char & character : message)
    {
        character ^= offset;
    }
}

std::string readPoint()
{
    std::stringstream sstream;
    int x;
    int y;

    std::cout << "Input x: " << std::flush;
    std::cin >> x;
    std::cout << "Input y: " << std::flush;
    std::cin >> y;

    sstream << x << " " << y;

    return sstream.str();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "crypto_publisher");
    ros::NodeHandle nodeHandle;
    ros::Publisher publisher = nodeHandle.advertise<ros_lab_2_msg::CryptoMessage>("crypto_messages", 100);

    unsigned int seed = (unsigned int) std::chrono::steady_clock::now().time_since_epoch().count();
    std::default_random_engine randomEngine(seed);
    std::uniform_int_distribution<uint8_t> distribution;

    ros_lab_2_msg::CryptoMessage cryptoMessage;
    while (ros::ok())
    {
        cryptoMessage.message = readPoint();
        cryptoMessage.cryptoMaterial = distribution(randomEngine);

        ROS_INFO("Decrypted message : %s", cryptoMessage.message.c_str());
        ROS_INFO("Crypto material: %d", cryptoMessage.cryptoMaterial);
        encryptMessage(cryptoMessage.message, cryptoMessage.cryptoMaterial);
        ROS_INFO("Encrypted message : %s", cryptoMessage.message.c_str());

        publisher.publish(cryptoMessage);
        ROS_INFO("=================================================");
    }
    return 0;
}