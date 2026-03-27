#pragma once
// Main header 
#include "storage.h"
#include "datatypes.h"

enum class AppRet {
    ERROR,
    OK
};


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

    // AppRet handleTapEvent(); // A user somewhere tapped a card
    // AppRet updateItemStatus(user& user, )


    User admin_user;
private:
    user_map_t user_map = {};
    storage_map_t storage_map = {};

    // Used on startup, fills up the user map
    AppRet load_all_user_data();


};


// How does device send data to server?
// Req: Type of signal, 

AppRet App::load_all_user_data()
{
    for (auto pair : user_map)
    {
        // Fill up storage map
        // UserStorage storage(pair.second);
        storage_map.try_emplace(pair.first, UserStorage(pair.second));
    }

    // for (auto pair : storage_map)
    // {
    //     pair.second
    // }
    return AppRet::OK;
}
