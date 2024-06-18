#include <string>
#include <vector>
#include <algorithm>
#include <sstream>
#include <cmath>
#include <memory>

// Function to find all indexes of a substring in a string
std::vector<int> find_all_indexes(const std::string& str, const std::string& substring) {
    std::vector<int> indexes;
    size_t start = 0;
    while (true) {
        start = str.find(substring, start);
        if (start == std::string::npos) {
            break;
        }
        indexes.push_back(start);
        start += substring.length();  // Move to the next possible start position
    }
    return indexes;
}

// Function to read messages from buffer between start_marker and end_marker
std::vector<std::string> read_messages_from_buffer(const std::string& buffer, const std::string& start_marker, const std::string& end_marker) {
    std::vector<int> start_markers = find_all_indexes(buffer, start_marker);
    std::vector<int> end_markers = find_all_indexes(buffer, end_marker);

    // Combine and sort the markers
    std::vector<std::pair<int, std::string>> start_and_end_markers;
    for (int index : start_markers) {
        start_and_end_markers.push_back({index, start_marker});
    }
    for (int index : end_markers) {
        start_and_end_markers.push_back({index, end_marker});
    }
    std::sort(start_and_end_markers.begin(), start_and_end_markers.end());

    std::vector<std::string> messages;
    bool starter = false;
    int start_i = -1;

    for (const auto& [index, marker] : start_and_end_markers) {
        if (marker == start_marker && !starter) {
            starter = true;
            start_i = index + start_marker.length();
        } else if (marker == end_marker && starter) {
            starter = false;
            messages.push_back(buffer.substr(start_i, index - start_i));
            start_i = -1;
        }
    }

    return messages;
}

// Function to get the latest message from buffer
std::string get_latest_message_from_buffer(const std::string& buffer, const std::string& start_marker, const std::string& end_marker) {
    std::vector<std::string> messages = read_messages_from_buffer(buffer, start_marker, end_marker);
    if (messages.empty()) {
        return "";
    }
    return messages.back();
}

// This strips a string and removes whitespaces and newlines
std::string strip(const std::string& str) {
    auto start = std::find_if_not(str.begin(), str.end(), [](int ch) {
        return std::isspace(ch);
    });

    auto end = std::find_if_not(str.rbegin(), str.rend(), [](int ch) {
        return std::isspace(ch);
    }).base();

    if (start >= end) {
        return "";  // return empty string if no non-whitespace character is found
    }

    return std::string(start, end);
}

std::string boolToString(bool b)
{
  return b ? "true" : "false";
}

std::string intToString(int value) {
    std::stringstream ss;
    ss << value;
    return ss.str();
}

std::string doubleToString(double value) {
    std::stringstream ss;
    ss << value;
    return ss.str();
}


int stringToInt(const std::string& str) {
    std::stringstream ss(str);
    int result;
    ss >> result;
    if (ss.fail()) {
        throw ("Conversion failed: stringToInt");
    }
    return result;
}


float intToFloat(int value) {
    std::stringstream ss;
    ss << value;
    float result;
    ss >> result;
    return result;
}

int floatToInt(float value) {
    std::stringstream ss;
    ss << value;
    int result;
    ss >> result;
    return result;
}

double intToDouble(int value) {
    std::stringstream ss;
    ss << value;
    double result;
    ss >> result;
    return result;
}

int doubleToInt(double value) {
    std::stringstream ss;
    ss << value;
    int result;
    ss >> result;
    return result;
}

bool intToBool(int value) {
    return value != 0;
}

int boolToInt(bool value) {
    return value ? 1 : 0;
}

double to_rad(double deg){
    return deg * (M_PI / 180.0);
}

double to_deg(double radians){
    return radians * (180.0 / M_PI);
}
