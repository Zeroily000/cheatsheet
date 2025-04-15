#include <Eigen/Core>
#include <iostream>

struct Foo1 {
  uint8_t a;
  Eigen::Matrix<double, 2, 1> b;
};

struct Foo2 {
  uint8_t a;
  Eigen::Matrix<double, 2, 1, Eigen::DontAlign> b;
};

struct __attribute__((__packed__)) Foo3 {
  uint8_t a;
  Eigen::Matrix<double, 2, 1> b;
};

int main() {
  std::cout << sizeof(Foo1) << std::endl;
  std::cout << sizeof(Foo2) << std::endl;
  std::cout << sizeof(Foo3) << std::endl;
  return 0;
}
