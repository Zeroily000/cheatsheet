#include <iostream>

#include "greet.h"
#include "modules/hello/hello.h"
#include "modules/world/world.h"

Greet::Greet() = default;
Greet::~Greet() = default;

void Greet::operator()() {
    Hello hello;
    World world;
    std::cout << hello() << " " << world() << std::endl;
}