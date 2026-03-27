# common_lib/CMakeLists.txt

if(ESP_PLATFORM)
    # ── ESP-IDF build ──────────────────────────────────────────────
    idf_component_register(
        SRCS        "src/common_lib.c"
        INCLUDE_DIRS "include"
    )
else()
    # ── Standard CMake build ───────────────────────────────────────
    cmake_minimum_required(VERSION 3.16)
    project(common_lib)

    add_library(common_lib STATIC
        src/common_lib.c
    )

    target_include_directories(common_lib PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )

    # Optional: install rules for find_package() support
    install(TARGETS common_lib EXPORT common_libTargets)
    install(DIRECTORY include/ DESTINATION include)
endif()