#include "modules/greet/greet.h"

#include <iostream>

#include "modules/hello/hello.h"
#include "modules/world/world.h"

Greet::Greet() = default;
Greet::~Greet() = default;

void Greet::operator()() const {
    Hello hello;
    World world;
    std::cout << hello() << " " << world() << std::endl;
}
