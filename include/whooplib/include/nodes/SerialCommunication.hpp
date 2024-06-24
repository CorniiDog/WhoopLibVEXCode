/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       SerialCommunication.hpp                                       */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  A keep-alive system for the Jetson Nano                   */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef SERIAL_COMMUNICATION_HPP
#define SERIAL_COMMUNICATION_HPP

#include "whooplib/include/nodes/NodeManager.hpp"
#include "whooplib/include/nodes/BufferNode.hpp"
#include <unordered_map>
#include <string>
#include <functional>
#include <memory> // For std::unique_ptr

class SerialCommunication: public ComputeNode{
private:
    std::unique_ptr<Messenger> keepalive_messenger = nullptr; // Handles messaging for pose data from Jetson Nano
    int time_waited_ms = 0;
    void setup_messenger(BufferNode* bufferSystem, const std::string& pose_stream);
    void _on_message_received(std::string message);

    int keep_alive_time_seconds;
public:
    SerialCommunication(BufferNode* bufferSystem, std::string communication_stream, int keep_alive_time_seconds);

    void reboot_jetson();
    void shutdown_jetson();

    /**
     * Processes messages and manages buffer space.
     */
    void __step() override;  // Protected helper function for processing steps
};

#endif