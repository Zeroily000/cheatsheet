#include "world.h"

World::World() = default;
World::~World() = default;

std::string World::operator()() const {
    return "world";
}
