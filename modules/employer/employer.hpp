#pragma once

#include "absl/status/statusor.h"
#include "boost/multi_index_container.hpp"
#include "boost/multi_index/hashed_index.hpp"
#include "boost/multi_index/indexed_by.hpp"
#include "boost/multi_index/member.hpp"
#include "boost/multi_index/ordered_index.hpp"

#include "modules/common/id_generator.hpp"
#include "modules/employee/employee.hpp"


class Employer {
 public:
  Employer();

  int AddEmployee(std::string const & name);

  absl::StatusOr<Employee> const GetEmployee(int id) const;

  absl::StatusOr<Employee> const GetEmployee(std::string const & name) const;

 private:
  common::IdGenerator<int> id_generator_;
  boost::multi_index_container<
    Employee,
    boost::multi_index::indexed_by<
      boost::multi_index::ordered_unique<boost::multi_index::member<Employee, int, &Employee::id>>,
      boost::multi_index::hashed_unique<boost::multi_index::member<Employee, std::string, &Employee::name>>
    >
  > employees_;
};
