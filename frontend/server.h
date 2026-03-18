#pragma once
// Main header 
// #include "database.h"
#include "storage.h"
#include "datatypes.h"


class App {
public:
    App() {
        // Reconstruct everything from files?
        // To start, create admin account
        admin_user.name = "admin";
        auto err = user_map.emplace(admin_user.name, admin_user);
        
        printf("Admin user: %s \n", user_map["admin"].name.c_str());
        UserStorage admin_storage(admin_user);

    }

private:
    User admin_user;
    user_map_t user_map = {};


};