#ifndef BUFFER_NODE_HPP
#define BUFFER_NODE_HPP

#include "whooplib/include/nodes/NodeManager.hpp"
#include <unordered_map>
#include <string>

// Declaration of BufferNode class
class BufferNode : public ComputeNode {
protected:
    // Upon initialization
    int max_buffer_size;
    std::string serial_conn;

    // Additional modifiables
    std::string my_buffer = "";  // Global buffer to store USB input data
    std::vector<std::string> registered_streams;
    std::unordered_map<std::string, std::string> messages;
public:
    bool debug_mode;

    BufferNode(int maxBufferSize=512, bool debugMode=false, std::string connection="/dev/serial1");  // Constructor declaration
    void register_stream(std::string stream); // Registers a stream for listening
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

    Messenger(BufferNode* bufferSystem, std::string stream, bool deleteAfterRead=false);
    void send(std::string message); // Send message to stream
    std::string read(); // Receive message from stream
};


#endif // BUFFER_NODE_H

