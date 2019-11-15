#include <iostream>

#include "hello_library.h"

int main() {
  std::cout << "Hello Bazel!" << std::endl;
  lib::print_hello();
  return 0;
}
