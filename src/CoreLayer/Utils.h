#pragma once

#include <cstdlib>
#include <iostream>

#define WARN_EXIT(TYPE, STR)                                          \
    std::cerr << TYPE << ": " << __FILE__ << " " << STR << std::endl; \
    exit(-1)

#define UNIMPLEMENTED(STR) WARN_EXIT("unimplemented", STR)

#define UNREACHABLE(STR) WARN_EXIT("unreachable", STR)
