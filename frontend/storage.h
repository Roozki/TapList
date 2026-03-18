#pragma once
#include <fstream>
#include "datatypes.h"

class NonVolatileMemory {
public:
    NonVolatileMemory(user_map_t& user_map) : user_map(user_map) {

    }
private:
user_map_t& user_map;
};