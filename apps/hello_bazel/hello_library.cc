#include "hello_library.h"

#include <iostream>

void lib::print_hello() {
  std::cout << "Hello Bazel, from my library!" << std::endl;
}
