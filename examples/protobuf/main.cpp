#include <iostream>

#include "examples/protobuf/foo.pb.h"

int main() {
  example::Foo foo;
  foo.set_id(1);
  foo.set_name("A");
  std::cout << foo.DebugString() << std::endl;
  return 0;
}
