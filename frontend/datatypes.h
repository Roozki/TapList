#pragma once
#include <string>
#include <cstdint>
#include <unordered_map>


enum class ItemStatus {
    unregistered, // Item not inited
    in_stock, // Item is in the kitchen
    out_of_stock // Item is out of stock or will be soon
};

struct Item {
    // std::string name = "UNAMED_ITEM";
    uint32_t card_uid = 0;
    ItemStatus status = ItemStatus::unregistered;
};

struct User {
    // std::string username = "UNREGISTERED_USER";
    std::unordered_map<std::string, Item> item_map = {};
};