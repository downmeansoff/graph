cmake_minimum_required(VERSION 3.23)

PROJECT(graph CXX)
set(CMAKE_CXX_STANDARD 23)


# Специфичные настройки для компилятора Visual Studio
if(MSVC)
    # Генерируем статическую заглушку для динамических библиотек
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS true)

    # Включаем высокий уровень предупреждений
    if(CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
        string(REGEX REPLACE "/W[0-4]" "/W4" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
    endif()
endif()

include(FetchContent)
FetchContent_Declare(googletest URL https://github.com/google/googletest/archive/6b63c98ac43efc992122f1da12aaf0a0e0658206.zip)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_library(graph SHARED "include/graph.cc")
target_include_directories(graph PUBLIC "include")

add_executable(graph_tests "tests/graph_tests.cc")
target_link_libraries(graph_tests graph GTest::gtest_main)

add_executable(task "task/task.cc")
target_link_libraries(task graph)
