#include <boost/core/demangle.hpp>
#include <iostream>

/**
 * @brief The number-sign or "stringizing" operator (#) converts macro parameters to string literals
 * without expanding the parameter definition. It's used only with macros that take arguments. If it
 * precedes a formal parameter in the macro definition, the actual argument passed by the macro
 * invocation is enclosed in quotation marks and treated as a string literal. The string literal
 * then replaces each occurrence of a combination of the stringizing operator and formal parameter
 * within the macro definition.
 *
 * @note White space that precedes the first token and follows the last token of the actual argument
 * is ignored. Any white space between the tokens in the actual argument is reduced to a single
 * white space in the resulting string literal. Further, if a character contained in the argument
 * usually requires an escape sequence when used in a string literal, for example, the quotation
 * mark (") or backslash (\) character, the necessary escape backslash is automatically inserted
 * before the character.
 */

#define F abc
#define B def
#define FB(arg) #arg
#define FB1(arg) FB(arg)

int main(int argc, char ** argv) {
  // FB(F B) -> #F B -> "F B"
  std::cout << "FB(F B) = \"" << FB(F B) << "\" (" << boost::core::demangle(typeid(FB(F B)).name())
            << ")" << std::endl;

  // FB1(F B) -> FB1(abc def) -> FB(abc def) -> #abc def -> "abc def"
  std::cout << "FB1(F B) = \"" << FB1(F B) << "\" ("
            << boost::core::demangle(typeid(FB1(F B)).name()) << ")" << std::endl;
  return 0;
}
