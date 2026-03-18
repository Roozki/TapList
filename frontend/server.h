#pragma once
// Main header 
// #include "database.h"
#include "storage.h"
#include "datatypes.h"


using user_map_t = std::unordered_map<std::string, User>;

class App {
public:
    App() {
        // Reconstruct everything from files?

    }

private:
    User admin_user;
    user_map_t user_map = {};


};