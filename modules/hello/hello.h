#pragma once

#include <string>

class Hello {
public:
    Hello();
    ~Hello();
    std::string const & operator()() const;

private:
    std::string value_;
};