
set(src
    physics/physics.cc
    physics/physics.h
)

source_group("" FILES ${src})
add_library(physics STATIC ${src})

target_include_directories(physics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(physics PUBLIC external::glm)

target_compile_features(physics PUBLIC cxx_std_17)
set_target_properties(physics PROPERTIES CXX_EXTENSIONS OFF)
add_library(physics::physics ALIAS physics)