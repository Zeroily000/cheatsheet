#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/indexed_by.hpp>
#include <boost/multi_index/mem_fun.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index_container.hpp>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

class Foo {
 public:
  Foo(std::string name, int id) : name{std::move(name)}, id{id} {}

  int GetId() const { return id; }

 public:
  std::string name;

 private:
  int id;
};

int main() {
  boost::multi_index_container<
      std::shared_ptr<Foo>,
      boost::multi_index::indexed_by<boost::multi_index::hashed_non_unique<
                                         boost::multi_index::member<Foo, std::string, &Foo::name>>,
                                     boost::multi_index::ordered_unique<
                                         boost::multi_index::const_mem_fun<Foo, int, &Foo::GetId>>>>
      foos;

  foos.insert(std::make_shared<Foo>("A", 1));
  foos.insert(std::make_shared<Foo>("B", 2));
  foos.insert(std::make_shared<Foo>("A", 3));

  auto const & index0{foos.get<0>()};
  auto const range{index0.equal_range("A")};
  for (auto itr{range.first}; itr != range.second; ++itr) {
    std::cout << (*itr)->GetId() << std::endl;
  }

  auto const & index1{foos.get<1>()};
  auto const itr{index1.find(2)};
  std::cout << (*itr)->name << std::endl;

  return 0;
}
