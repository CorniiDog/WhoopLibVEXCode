/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       BufferNode.cpp                                       */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  A Streamlined Communication System (like for Jetson nano) */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/include/nodes/BufferNode.hpp"
#include "whooplib/include/toolbox.hpp"

#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <cerrno>
#include <stdio.h>
#include <string>
#include <functional>

////////////////////////////////////////////////////////////////////////////////
// Buffer Node Class For Receiving Jetson Nano Stream
////////////////////////////////////////////////////////////////////////////////

// BufferNode class methods
BufferNode::BufferNode(int maxBufferSize, debugMode debugMode, std::string connection) : max_buffer_size(maxBufferSize), serial_conn(connection), debug_mode(debugMode) {}

void BufferNode::__step()
{
    ////////////////////////////////////////////////////////////////////////
    // Acquiring data
    // FILE *fp = fopen(serial_conn.c_str(), "r");
    FILE *fp = fopen(serial_conn.c_str(), "r");
    // If serial connection not established, don't continue
    if (!fp)
    {
        return;
    }

    // Get the file descriptor from the FILE* object
    int fd = fileno(fp);

    // Set the file descriptor to non-blocking mode
    int flags = fcntl(fd, F_GETFL, 0);

    // Failed to get file flags
    if (flags == -1)
    {
        return;
    }

    if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) == -1)
    {
        return; // Failed to set non-blocking
    }

    // Read buffer
    char buffer[max_buffer_size];
    std::string result;

    ssize_t read_bytes = read(fd, buffer, sizeof(buffer) - 1);

    if (read_bytes > 0)
    {
        buffer[read_bytes] = '\0'; // Null-terminate the string
        result = buffer;
    }
    else if (read_bytes == -1 && errno != EAGAIN && errno != EWOULDBLOCK)
    {
        // If error
        fclose(fp);
        return;
    }

    // Close stream
    fclose(fp);
    ////////////////////////////////////////////////////////////////////////
    // Applying data

    if (lock_ptr)
        lock_ptr->lock(); // Acquire the mutex

    my_buffer += result;
    if (my_buffer.size() > max_buffer_size)
    {
        my_buffer = my_buffer.substr(my_buffer.size() - max_buffer_size);
    }

    if (lock_ptr)
        lock_ptr->unlock(); // Release the mutex

    ////////////////////////////////////////////////////////////////////////
    // Applying messages to registered streams
    for (size_t i = 0; i < registered_messengers.size(); ++i)
    {
        std::string latest_msg = get_latest_message_from_buffer(my_buffer, "[<" + registered_messengers[i]->messenger_stream + ">]", "&=" + registered_messengers[i]->messenger_stream + "*$");
        if (latest_msg != "")
        {
            if (lock_ptr)
                lock_ptr->lock(); // Acquire the mutex

            messages[registered_messengers[i]->messenger_stream] = strip(latest_msg);

            if (lock_ptr)
                lock_ptr->unlock(); // Release the mutex

            for (size_t j = 0; j < registered_messengers[i]->callback_functions.size(); ++j)
            {
                if (debug_mode)
                {
                    registered_messengers[i]->callback_functions[j](latest_msg);
                }
                else
                {
                    try
                    {
                        registered_messengers[i]->callback_functions[j](latest_msg);
                    }
                    catch (const std::exception &e)
                    {
                        Brain.Screen.clearLine(1);
                        Brain.Screen.setCursor(1, 1);
                        Brain.Screen.print("Error: %s", e.what());
                    }
                }
            }
        }
    }
}

void BufferNode::register_stream(Messenger *messenger)
{
    registered_messengers.push_back(messenger);
}

std::string BufferNode::get_message(std::string stream, bool delete_after_read)
{
    if (messages.find(stream) != messages.end())
    {
        if (delete_after_read)
        {
            std::string msg = messages[stream];
            messages.erase(stream);
            return msg;
        }
        return messages[stream];
    }
    return "";
}

int BufferNode::send_message(std::string stream, std::string message, std::string end)
{

    std::string msg = ("[<" + stream + ">]" + message + "&=" + stream + "*$");
    const int msg_size = msg.size();

    if (lock_ptr)
        lock_ptr->lock(); // Acquire the mutex

    FILE *fp = fopen(serial_conn.c_str(), "w");

    // If serial connection not established, don't continue
    if (!fp)
        return 2;

    // Writing to serial connection
    if (fwrite(msg.c_str(), 1, msg_size, fp) != msg_size)
    {
        // If there is an issue, then close file and return
        fclose(fp); // Ensure the file is closed even if writing fails
        if (lock_ptr)
            lock_ptr->unlock(); // Release the mutex
        return 1;
    }

    // Always check the fclose result for errors
    if (fclose(fp) != 0)
    {
        if (lock_ptr)
            lock_ptr->unlock(); // Release the mutex
        return 3;
    }

    if (lock_ptr)
        lock_ptr->unlock(); // Release the mutex

    // Everything went smoothly, return 0
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Messenger Class for Simplified Functionality
////////////////////////////////////////////////////////////////////////////////

Messenger::Messenger(BufferNode *bufferSystem, std::string stream, deleteAfterRead deleteAfterRead) : messenger_stream(stream), delete_after_read(deleteAfterRead)
{
    buffer_system = bufferSystem;
    buffer_system->register_stream(this);
}

void Messenger::send(std::string message)
{
    buffer_system->send_message(messenger_stream, message);
}

std::string Messenger::read()
{
    return buffer_system->get_message(messenger_stream, delete_after_read);
}

void Messenger::on_message(std::function<void(std::string)> callback)
{
    callback_functions.push_back(callback);
}