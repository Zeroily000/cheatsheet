#pragma once

#include <string>

class World {
public:
    World();
    ~World();
    std::string operator()();
};