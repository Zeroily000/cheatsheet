#pragma once

#include <string>

class World {
public:
    World();
    ~World();
    std::string const & operator()() const;

private:
    std::string value_;
};