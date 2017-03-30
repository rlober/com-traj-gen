cmake_minimum_required(VERSION 2.8.11)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(YARP REQUIRED)
find_package(OcraRecipes REQUIRED)



include_directories(${YARP_INCLUDE_DIRS}
                    ${OcraRecipes_INCLUDE_DIRS})

link_directories(${YARP_LIBRARY_DIRS} ${OcraRecipes_LIBRARY_DIRS})

add_executable(com-traj-gen main.cpp)
target_link_libraries(com-traj-gen ${YARP_LIBRARIES} ${OcraRecipes_LIBRARIES})

install(TARGETS com-traj-gen DESTINATION bin)
include(AddUninstallTarget)

add_custom_target(  atom-files ALL
                        DEPENDS com-traj-gen
                        COMMAND ${CMAKE_COMMAND} -E copy
                        ${CMAKE_CURRENT_BINARY_DIR}/compile_commands.json
                        ${CMAKE_SOURCE_DIR}
                        COMMENT "Copying 'compile_commands.json' to '${CMAKE_SOURCE_DIR}'"
                     )
