#include "modules/world/world.hpp"

World::World() : value_("world") {}
World::~World() = default;

std::string const & World::operator()() const {
    return value_;
}
