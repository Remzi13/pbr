cmake_minimum_required(VERSION 3.10)

project(pbr CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SRC
    src/vector.h
    src/matrix.h
    src/scene.h
    src/scene.cpp
    src/utils.h
    src/utils.cpp
    src/concurrency.h
    src/concurrency.cpp
    src/bvh.h
    src/bvh.cpp
    src/brdf.h
    src/brdf.cpp
    src/gltf.h
    src/gltf.cpp
    main.cpp
)

add_executable(pbr ${SRC})

