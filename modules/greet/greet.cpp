#include "modules/greet/greet.hpp"

#include <iostream>

#include "modules/hello/hello.hpp"
#include "modules/world/world.hpp"

Greet::Greet() = default;
Greet::~Greet() = default;

void Greet::operator()() const {
    Hello hello;
    World world;
    std::cout << hello() << " " << world() << std::endl;
}
