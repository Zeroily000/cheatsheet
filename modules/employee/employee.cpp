#include "modules/employee/employee.hpp"

#include <utility>

Employee::Employee(int id, std::string name) : id(id), name(std::move(name)) {}
