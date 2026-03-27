#include <iostream>
#include <string>
#include <mutex>
#include "server.h"
#include "transcoder.h"

#include "../external_pkgs/cpp-httplib/httplib.h" // Gross include 

int main() {

    httplib::Server server;
    App app;

    int shared_value = 0;
    std::mutex value_mutex;

    // Serve files from current directory
    server.set_mount_point("/", ".");

    // server.Get("/value", [&](const httplib::Request&, httplib::Response& res) {
        
    //     std::lock_guard<std::mutex> lock(value_mutex);
    
    //     std::string json =
    //         "{\"value\": " + std::to_string(shared_value) + "}";
    
    //     res.set_content(json, "application/json");
    // });
    Item test_item;
    test_item.card_uid = 99;
    test_item.name = "MeowMeow";
    test_item.status = ItemStatus::unknown;
    app.admin_user.item_map.emplace("MeowMeow", test_item);

    // API endpoint returning JSON
    server.Get("/items", [&](const httplib::Request&, httplib::Response& res) {

        // std::vector<std::string>

        std::lock_guard<std::mutex> lock(value_mutex);

        std::string have_json, need_json;

        for (const auto& [key, item] : app.admin_user.item_map) {
            if (item.status == ItemStatus::in_stock) {
                if (!have_json.empty()) have_json += ",";
                have_json += "\"" + item.name + "\"";
            } else if (item.status == ItemStatus::out_of_stock) {
                if (!need_json.empty()) need_json += ",";
                need_json += "\"" + item.name + "\"";
            }
        }

std::string json = "{\"have\":[" + have_json + "],\"need\":[" + need_json + "]}";

        res.set_header("Access-Control-Allow-Origin", "*");
        res.set_content(json, "application/json");
    });

    // API endpoint updating value
    server.Post("/update", [&](const httplib::Request& req, httplib::Response& res) {

        // if (!req.has_param("x")) {
        //     res.status = 400;
        //     res.set_content("Missing parameter x\n", "text/plain");
        //     return;
        // }

        uint32_t new_value = 0;
        memcpy(&new_value, &req.body[0], sizeof(new_value));


        {
            std::lock_guard<std::mutex> lock(value_mutex);
            shared_value = new_value;
        }
    std::cout << "POST /update hit\n";
    std::cout << "req.body = [" << req.body << "]\n";
    std::cout << "body size = [" << req.body.length() << "]\n";

        // try {

        //     // int new_value = std::stoi(req.get_param_value("x")); // JSON or whatever
        //     int new_value = req.get_param_value("x"); // raw binary

        //     {
        //         std::lock_guard<std::mutex> lock(value_mutex);
        //         shared_value = new_value;
        //     }

        //     res.set_content("Value updated\n", "text/plain");

        // } catch (...) {

        //     res.status = 400;
        //     res.set_content("Invalid integer\n", "text/plain");

        // }
    });

    std::cout << "Server running at http://localhost:8080\n";

    // server.listen("127.0.0.1", 8080);
    server.listen("0.0.0.0", 8080);
    std::cout << "Server Stopped. \n";

}