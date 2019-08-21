#include <iostream>

#include "apps/hello_bazel/hello_library.h"

int main() {
  std::cout << "Hello dependency!" << std::endl;
  lib::print_hello();
  return 0;
}
