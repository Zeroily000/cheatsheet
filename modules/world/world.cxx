#include "modules/world/world.h"

World::World() : value_("world") {}
World::~World() = default;

std::string const & World::operator()() const {
    return value_;
}
