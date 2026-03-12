#include <iostream>
#include <string>
#include <mutex>

#include "../external_pkgs/cpp-httplib/httplib.h"

int main() {

    httplib::Server server;

    int shared_value = 0;
    std::mutex value_mutex;

    // Serve files from current directory
    server.set_mount_point("/", ".");

    // API endpoint returning JSON
    server.Get("/value", [&](const httplib::Request&, httplib::Response& res) {

        std::lock_guard<std::mutex> lock(value_mutex);

        std::string json =
            "{\"value\": " + std::to_string(shared_value) + "}";

        res.set_content(json, "application/json");
    });

    // API endpoint updating value
    server.Post("/update", [&](const httplib::Request& req, httplib::Response& res) {

        // if (!req.has_param("x")) {
        //     res.status = 400;
        //     res.set_content("Missing parameter x\n", "text/plain");
        //     return;
        // }
    int new_value = std::stoi(req.body);

        {
            std::lock_guard<std::mutex> lock(value_mutex);
            shared_value = new_value;
        }
    std::cout << "POST /update hit\n";
    std::cout << "req.body = [" << req.body << "]\n";
    std::cout << "body size = [" << sizeof(req.body) << "]\n";

        try {

            int new_value = std::stoi(req.get_param_value("x"));

            {
                std::lock_guard<std::mutex> lock(value_mutex);
                shared_value = new_value;
            }

            res.set_content("Value updated\n", "text/plain");

        } catch (...) {

            res.status = 400;
            res.set_content("Invalid integer\n", "text/plain");

        }
    });

    std::cout << "Server running at http://localhost:8080\n";

    server.listen("0.0.0.0", 8080);
}