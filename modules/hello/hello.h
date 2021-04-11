#pragma once

#include <string>

class Hello {
public:
    Hello();
    ~Hello();
    std::string operator()() const;
};