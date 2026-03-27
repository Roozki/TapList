#pragma once
#include <string>
#include <cstdint>
#include <unordered_map>
#include <fstream>
// #include "storage.h"
// #include "string.h"
#include <vector>

enum class ItemStatus {
    unregistered, // Item not inited
    in_stock, // Item is in the kitchen
    out_of_stock, // Item is out of stock or will be soon
    unknown, // Error State
    num_item_status
};

struct Item {
    std::string name = "UNAMED_ITEM";
    uint32_t card_uid = 0;
    ItemStatus status = ItemStatus::unregistered;
};

struct User {
    std::string name = "UNREGISTERED_USER";
    std::unordered_map<std::string, Item> item_map = {};
};

using user_map_t = std::unordered_map<std::string, User>;
