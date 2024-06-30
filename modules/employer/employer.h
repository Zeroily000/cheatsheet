#pragma once

#include <boost/multi_index_container.hpp>
#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/indexed_by.hpp>

#include "modules/employee/employee.h"


class Employer {
 public:
  Employer();

  void AddEmployee(int id, std::string const & name);

  Employee const & GetEmployee(int id) const;

  Employee const & GetEmployee(std::string const & name) const;

 private:
  boost::multi_index_container<
    Employee,
    boost::multi_index::indexed_by<
      boost::multi_index::ordered_unique<boost::multi_index::member<Employee, int, &Employee::id>>,
      boost::multi_index::hashed_unique<boost::multi_index::member<Employee, std::string, &Employee::name>>
    >
  > employees_;
};
