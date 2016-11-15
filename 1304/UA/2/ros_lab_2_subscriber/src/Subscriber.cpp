#include <ros/ros.h>
#include <chrono>
#include "ros_lab_2_msg/CryptoMessage.h"

void decryptMessage(std::string& message, uint8_t offset)
{
    for (char & character : message)
    {
        character ^= offset;
    }
}

std::pair<int32_t, int32_t> parsePoint(const std::string& message)
{
    size_t offset = message.find(' ');
    return std::make_pair<int32_t, int32_t>(
            std::stoi(std::string(message.begin(), message.begin() + offset)),
            std::stoi(std::string(message.begin() + offset, message.end()))
    );
};

void handleCryptoMessage(const boost::shared_ptr<const ros_lab_2_msg::CryptoMessage>& cryptoMessage)
{
    ROS_INFO("Crypto message : %s", cryptoMessage->message.c_str());
    ROS_INFO("Crypto material: %d", cryptoMessage->cryptoMaterial);
    std::string message = cryptoMessage->message;
    decryptMessage(message, cryptoMessage->cryptoMaterial);
    ROS_INFO("Decrypted message : %s", message.c_str());
    auto point = parsePoint(message);
    ROS_INFO("Target x: %d", point.first);
    ROS_INFO("Target y: %d", point.second);
    ROS_INFO("=================================================");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "crypto_subscriber");
    ros::NodeHandle nodeHandle;
    ros::Subscriber subscriber = nodeHandle.subscribe<ros_lab_2_msg::CryptoMessage>("crypto_messages", 100, &handleCryptoMessage);

    ros::Rate loop_rate(60);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    };
    return 0;
}