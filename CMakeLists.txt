cmake_minimum_required(VERSION 3.14)

project(pbr CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# Fetch SDL2
FetchContent_Declare(
  SDL2
  GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
  GIT_TAG release-2.30.1 # Using a stable release tag
)
set(SDL_SHARED ON CACHE BOOL "" FORCE)
set(SDL_STATIC ON CACHE BOOL "" FORCE)
set(SDL_TEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(SDL2)

## Fetch ImGui
FetchContent_Declare(
  imgui
  URL https://github.com/ocornut/imgui/archive/refs/heads/docking.tar.gz
)
FetchContent_MakeAvailable(imgui)

# ImGui source files
set(IMGUI_SRC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl2.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_sdlrenderer2.cpp
)

include_directories("src" ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends)

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
    src/brdf.h
    src/brdf.cpp
    src/gltf.h
    src/gltf.cpp
    main.cpp
    ${IMGUI_SRC}
)

add_executable(pbr ${SRC})

# Find OpenGL (still might be useful for other things, but not strictly needed for SDL_Renderer)
find_package(OpenGL REQUIRED)

# Link libraries
target_link_libraries(pbr SDL2::SDL2 OpenGL::GL ${CMAKE_DL_LIBS})