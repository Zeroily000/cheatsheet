//module;
//#include <iostream>
// import <iostream>;
//#include <string_view>

export module foo;

export auto plus(auto x, auto y) -> decltype(x + y) {
  return x + y;
}

//export void Hello(std::string_view const & name) { std::cout << "Hello " << name << "!\n"; }
