#pragma once
#include <fstream>
#include "datatypes.h"

#define FILE_EXT ".taplist" // file extension

// static inline constexpr char[10] file_ext = ".taplist";
// Originally NonVolatileMemory

// Purpose: Store and retreive item list based on user name

class UserStorage {
public:
    UserStorage(User& user) : user(user) {

        filename = user.name + FILE_EXT;

        // See if file exhists yet
        file.open(filename);
        if(file.is_open())
        {
            printf("file exhists for %s", user.name.c_str());
        } else 
        {
            printf("file open failure for %s", user.name.c_str());
        }

    }
private:
    std::string filename = "";
    std::fstream file;
    User& user;
    
    
};