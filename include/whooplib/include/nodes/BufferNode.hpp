#ifndef BUFFER_NODE_HPP
#define BUFFER_NODE_HPP

#include "whooplib/include/nodes/NodeManager.hpp"
#include <unordered_map>
#include <string>
#include <functional>

enum deleteAfterRead{
    no_delete=true,
    yes_delete=false
};

enum debugMode{
    debug_enabled=true,
    debug_disabled=false
};

class Messenger; // Forward declaration

// Declaration of BufferNode class
class BufferNode : public ComputeNode {
protected:
    // Upon initialization
    int max_buffer_size;
    std::string serial_conn;

    // Additional modifiables
    std::string my_buffer = "";  // Global buffer to store USB input data
    std::vector<Messenger*> registered_messengers;
    std::unordered_map<std::string, std::string> messages;
public:
    bool debug_mode;

    BufferNode(int maxBufferSize=512, debugMode debugMode=debugMode::debug_disabled, std::string connection="/dev/serial1");  // Constructor declaration
    void register_stream(Messenger* messenger); // Registers a stream for listening
    std::string get_message(std::string stream, bool delete_after_read=false); // Receive a message from a stream from USB (returns empty string if nothing)
    int send_message(std::string stream, std::string message, std::string end="\n"); // Sends a message to a stream over USB
protected:
    void __step() override;  // Protected helper function for processing steps
};

class Messenger{
protected:
    BufferNode* buffer_system;
public:
    std::string messenger_stream;
    bool delete_after_read;
    std::vector<std::function<void(std::string)>> callback_functions;

    Messenger(BufferNode* bufferSystem, std::string stream, deleteAfterRead deleteAfterRead=deleteAfterRead::no_delete);
    void send(std::string message); // Send message to stream
    std::string read(); // Receive message from stream
    void on_message(std::function<void(std::string)> callback);
};


#endif // BUFFER_NODE_H

