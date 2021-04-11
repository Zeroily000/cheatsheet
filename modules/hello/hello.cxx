#include "hello.h"

Hello::Hello() = default;
Hello::~Hello() = default;

std::string Hello::operator()() {
    return "hello";
}