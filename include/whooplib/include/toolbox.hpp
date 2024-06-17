#ifndef TOOLBOX_HPP
#define TOOLBOX_HPP

#include <string>
#include <vector>

// Function to find all indexes of a substring in a string
std::vector<int> find_all_indexes(const std::string& str, const std::string& substring);

// Function to read messages from buffer between start_marker and end_marker
std::vector<std::string> read_messages_from_buffer(const std::string& buffer, const std::string& start_marker, const std::string& end_marker);

// Function to get the latest message from buffer
std::string get_latest_message_from_buffer(const std::string& buffer, const std::string& start_marker, const std::string& end_marker);

// This strips a string and removes whitespaces and newlines
std::string strip(const std::string& str);

// Conversions
std::string boolToString(bool b);
std::string intToString(int value);
int stringToInt(const std::string& str);
float intToFloat(int value);
int floatToInt(float value);
double intToDouble(int value);
int doubleToInt(double value);
bool intToBool(int value);
int boolToInt(bool value);

// Degrees to Radians conversions
double to_rad(double deg);
double to_deg(double rad);

#endif