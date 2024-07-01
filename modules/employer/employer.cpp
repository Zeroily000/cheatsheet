#include "modules/employer/employer.h"

Employer::Employer() : id_generator_(1) {}

int Employer::AddEmployee(std::string const & name) {
  int const id{id_generator_()};
  employees_.emplace(id, name);
  return id;
}

Employee const & Employer::GetEmployee(int id) const {
  auto const itr{employees_.get<0>().find(id)};
  return *itr;
}

Employee const & Employer::GetEmployee(std::string const & name) const {
  auto const itr{employees_.get<1>().find(name)};
  return *itr;
}
