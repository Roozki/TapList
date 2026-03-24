#pragma once
#include <fstream>
#include "datatypes.h"
#include <chrono>

#define FILE_EXT ".taplist" // file extension
#define DELIMITER ','

#define ITEM_IN_STOCK_KEY_STR "IN_STOCK"
#define ITEM_OUT_OF_STOCK_KEY_STR "OUT_OF_STOCK"



// #define END_OF_LINE ','

// static inline constexpr char[10] file_ext = ".taplist";
// Originally NonVolatileMemory

// Purpose: Store and retreive item list based on user name

enum class StorageRet {
    OK,
    NOT_OK
};

enum class FileStatus {
    OPEN,
    CLOSED,
    ERROR
};

class UserStorage {
public:
    UserStorage(User& user) : user(user) {

        filename = user.name + FILE_EXT;

        // Read from file

        // Stored as csv
        std::string line;
        int num_invalid_entries = 0;
        auto t1 = std::chrono::steady_clock::now();
        while (std::getline(file, line)) {
            size_t pos = 0;
            if ((pos = line.find(DELIMITER)) != std::string::npos) {
                std::string found_item_name = "";
                std::string found_item_status_str = "";
                found_item_name = line.substr(0, pos);
                found_item_status_str = line.substr(pos + 1, line.length());
                ItemStatus item_status = ItemStatus::unknown;
                if(decode_item_status(found_item_status_str, item_status) != StorageRet::OK)
                {
                    printf("FOR %s:%s, Item status string invalid \n", user.name.c_str(), found_item_name.c_str());
                    num_invalid_entries++;
                }

                // Construct item that was stored in file
                Item item = {};
                item.card_uid = 0; //TODO Figure this part out.
                item.name = found_item_name;
                item.status = item_status;
                // Load into item hashmap
                user.item_map.emplace(found_item_name, item);
                printf("FOR %s: Found item [%s] with status [%s] \n", user.name.c_str(), found_item_name.c_str(), found_item_status_str.c_str());
            } else {
                printf("FOR %s, no delimeter found in line! \n", user.name.c_str());
            }
        }
        auto t2 = std::chrono::steady_clock::now();

        printf("All items found for  %s. Hashmap contains %li entries, including %i invalid entries \n", user.name.c_str(), user.item_map.size(), num_invalid_entries);
        std::chrono::steady_clock::duration elapsed_time = t2 - t1;
        // std::chrono::milliseconds ms =
        // std::chrono::duration_cast<double>(elapsed_time);
        std::chrono::duration<double> s = elapsed_time;
        printf("Took %0.8f seconds to load \n", s);
    }


    // Read and load user list, to store in user reference
    StorageRet load_items(); 
    StorageRet open_user_file();
    StorageRet close_user_file();
    
private:
    FileStatus file_status = FileStatus::CLOSED;
    std::string filename = "";
    std::fstream file;
    User& user;
    static StorageRet decode_item_status(const std::string item_status_str, ItemStatus& item_status);


    
    
};

StorageRet UserStorage::decode_item_status(const std::string item_status_str, ItemStatus& item_status)
{
    item_status = ItemStatus::unknown;
    if(item_status_str == ITEM_IN_STOCK_KEY_STR)
    {
        item_status = ItemStatus::in_stock;
        return StorageRet::OK;
    }

    if(item_status_str == ITEM_OUT_OF_STOCK_KEY_STR)
    {
        item_status = ItemStatus::out_of_stock;
        return StorageRet::OK;
    }

    //TODO handle items in registration
    item_status = ItemStatus::unknown;
    return StorageRet::NOT_OK;
}

// Finds and opens user file
StorageRet UserStorage::open_user_file()
{
    if(file_status == FileStatus::OPEN)
    {
        // Log(Error, "User file attemped to open while already open.")
        return StorageRet::NOT_OK;
    }

    //TODO store if file should or should not exhist within user.
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
    return StorageRet::OK;
}

StorageRet UserStorage::load_items()
{
    //TODO CRC

    return StorageRet::OK;
}