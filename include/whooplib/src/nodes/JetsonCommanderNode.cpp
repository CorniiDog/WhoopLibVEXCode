/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       JetsonCommanderNode.cpp                                   */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  A keep-alive system for the Jetson Nano                   */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/include/nodes/JetsonCommanderNode.hpp"
#include "whooplib/include/toolbox.hpp"
#include <unordered_map>
#include <string>
#include <functional>

void JetsonCommander::setup_messenger(BufferNode* bufferSystem, const std::string& pose_stream){
    keepalive_messenger = std::make_unique<Messenger>(bufferSystem, pose_stream, deleteAfterRead::no_delete);
    keepalive_messenger->on_message(std::bind(&JetsonCommander::_on_message_received, this, std::placeholders::_1));
}

void JetsonCommander::_on_message_received(std::string message){
    if(message == "Hello"){
        keepalive_messenger->send(intToString(keep_alive_time_seconds) + " " + "Initialize");
    }
}


JetsonCommander::JetsonCommander(BufferNode* bufferSystem, std::string communication_stream, int keep_alive_time_seconds){
    this->keep_alive_time_seconds = keep_alive_time_seconds;
    setup_messenger(bufferSystem, communication_stream);
    this->set_step_time(1000); // Configure step time in milliseconds
}

void JetsonCommander::reboot_jetson(){
    keepalive_messenger->send("Reboot");
}

void JetsonCommander::shutdown_jetson(){
    keepalive_messenger->send("Shutdown");
}


void JetsonCommander::__step(){
    keepalive_messenger->send(intToString(keep_alive_time_seconds));
}