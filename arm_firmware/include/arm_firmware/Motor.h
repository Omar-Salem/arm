//
// Created by omar on 1/20/24.
//


#include <string>

class Motor {
public:
    double position_state;
    double position_command;
    std::string name;

    explicit Motor(const std::string &name) : name(name) {}
};
