#include "modules/employer/employer.hpp"

Employer::Employer() : id_generator_(1) {}

int Employer::AddEmployee(std::string const & name) {
  int const id{id_generator_()};
  employees_.emplace(id, name);
  return id;
}

absl::StatusOr<Employee> const Employer::GetEmployee(int id) const {
  auto const & id_index{employees_.get<0>()};
  auto const itr{id_index.find(id)};
  if (itr == id_index.cend()) {
    return absl::NotFoundError("Employee does not exist");
  }
  return *itr;
}

absl::StatusOr<Employee> const Employer::GetEmployee(std::string const & name) const {
  auto const & name_index{employees_.get<1>()};
  auto const itr{name_index.find(name)};
  if (itr == name_index.cend()) {
    return absl::NotFoundError("Employee does not exist");
  }
  return *itr;
}
