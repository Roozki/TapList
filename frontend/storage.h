#pragma once
#include <fstream>
#include "datatypes.h"

#define FILE_EXT ".taplist" // file extension
#define DELIMITER ','
// #define END_OF_LINE ','

// static inline constexpr char[10] file_ext = ".taplist";
// Originally NonVolatileMemory

// Purpose: Store and retreive item list based on user name

class UserStorage {
public:
    UserStorage(User& user) : user(user) {

        filename = user.name + FILE_EXT;

        // See if file exhists yet
        file.open(filename, std::ios::out | std::ios::in);
        if(file.is_open())
        {
            printf("file exhists for %s \n", user.name.c_str());
        } else 
        {
            printf("File not found for %s. Creating one now... \n", user.name.c_str());
            file.open(filename, std::ios::out | std::ios::in | std::ios::trunc);
            file << "FILE_START \n";
        }

        // Read from file

        // Stored as csv
        std::string line;
        while (std::getline(file, line)) {
            size_t pos = 0;
            if ((pos = line.find(DELIMITER)) != std::string::npos) {
                std::string found_item_name = "";
                std::string found_item_status = "";
                found_item_name = line.substr(0, pos);
                found_item_status = line.substr(pos + 1, line.length());
                printf("FOR %s: Found item [%s] with status [%s] \n", user.name.c_str(), found_item_name.c_str(), found_item_status.c_str());
            } else {
                printf("FOR %s, no delimeter found in line! \n", user.name.c_str());
            }
        }
        printf("All items found for  %s \n", user.name.c_str());
    }
private:
    std::string filename = "";
    std::fstream file;
    User& user;


    
    
};