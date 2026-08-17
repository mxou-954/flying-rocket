cmake_minimum_required(VERSION 3.15)
project(m31_simulation LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# --- Bibliothèque coeur (physique + config), partagée par l'exe et les tests ---
add_library(m31_core
    src/simulation.cpp
    src/config.cpp
)
target_include_directories(m31_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

if(MSVC)
    target_compile_options(m31_core PRIVATE /W4)
else()
    target_compile_options(m31_core PRIVATE -Wall -Wextra)
endif()

# --- Exécutable principal ---
add_executable(m31 src/main.cpp)
target_link_libraries(m31 PRIVATE m31_core)

# --- Tests ---
enable_testing()
add_executable(m31_tests tests/tests.cpp)
target_link_libraries(m31_tests PRIVATE m31_core)
add_test(NAME unit_tests COMMAND m31_tests)
