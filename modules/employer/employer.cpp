#include "modules/employer/employer.h"

Employer::Employer() = default;

void Employer::AddEmployee(int id, std::string const & name) {
  employees_.emplace(id, name);
}

Employee const & Employer::GetEmployee(int id) const {
  auto const itr{employees_.get<0>().find(id)};
  return *itr;
}

Employee const & Employer::GetEmployee(std::string const & name) const {
  auto const itr{employees_.get<1>().find(name)};
  return *itr;
}
