cmake_minimum_required(VERSION 3.0.2)
project(moveit_pnp_server_apps)
add_compile_options(-std=c++14)

###########
## Build ##
###########

file(GLOB COMMON_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/*.*
)

macro(create_app name)
    add_executable(${name}
        ${name}.cpp
        ${COMMON_SOURCES}
    )
    target_link_libraries(${name}
        ${catkin_LIBRARIES}
    )
    target_include_directories(${name}
        PUBLIC
        ${catkin_INCLUDE_DIRS}
    )
    add_dependencies(${name}
        ${catkin_EXPORTED_TARGETS}
    )
endmacro(create_app)

# to add a new executable... simply add `create_app(<exe_name>)`