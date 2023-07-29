#pragma once

#include <cstdlib>
#include <iostream>

#define WARN_EXIT(TYPE, STR)                                              \
    do {                                                                  \
        std::cerr << TYPE << ": " << __FILE__ << " " << STR << std::endl; \
        exit(-1);                                                         \
    } while (0);

#define UNIMPLEMENTED(STR) WARN_EXIT("unimplemented", STR)

#define UNREACHABLE(STR) WARN_EXIT("unreachable", STR)
