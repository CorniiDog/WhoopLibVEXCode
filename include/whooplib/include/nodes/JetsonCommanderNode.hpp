/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       JetsonCommanderNode.hpp                                   */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  A keep-alive system for the Jetson Nano                   */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef JETSON_COMMANDER_HPP
#define JETSON_COMMANDER_HPP

#include "whooplib/include/nodes/NodeManager.hpp"
#include "whooplib/include/nodes/BufferNode.hpp"
#include "whooplib/include/devices/WhoopController.hpp"
#include <unordered_map>
#include <string>
#include <functional>
#include <memory> // For std::unique_ptr

enum jetsonCommunication{
    enable_comms=true,
    disable_comms=false
};

class JetsonCommander: public ComputeNode{
private:
    std::unique_ptr<Messenger> keepalive_messenger = nullptr; // Handles messaging for pose data from Jetson Nano
    int time_waited_ms = 0;

    void setup_messenger(BufferNode* bufferSystem, const std::string& pose_stream);
    void _on_message_received(std::string message);
    WhoopController* controller_for_messages;

    int raw_connected = 5;

    int keep_alive_time_seconds;

    bool comms_disabled = false;
public:
    bool connected = false;
    JetsonCommander(WhoopController* controller_for_messages, BufferNode* bufferSystem, std::string communication_stream, int keep_alive_time_seconds, int step_time_ms, jetsonCommunication enable_jetson_comms);

    void reboot_jetson();
    void shutdown_jetson();

    bool is_connected_to_jetson();

    /**
     * Processes messages and manages buffer space.
     */
    void __step() override;  // Protected helper function for processing steps
};

#endif // JETSON_COMMANDER_HPP