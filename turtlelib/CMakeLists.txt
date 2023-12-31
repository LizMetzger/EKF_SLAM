cmake_minimum_required(VERSION 3.9)
project(turtlelib)

find_package(Armadillo REQUIRED)
add_executable(frame_main src/frame_main.cpp)
# create the turtlelib library
include_directories(include/turtlelib)
set(HEADER_FILES include/turtlelib/diff_drive.hpp)
add_library(turtlelib src/rigid2d.cpp src/diff_drive.cpp  src/make_circles.cpp ${HEADER_FILES})
target_link_libraries(turtlelib ${ARMADILLO_LIBRARIES})
# The add_library function just added turtlelib as a "target"
# A "target" is a name that CMake uses to refer to some type of output
# In this case it is a library but it could also be an executable or some other items

target_link_libraries(frame_main turtlelib ${ARMADILLO_LIBRARIES})
# Public include directories can be used by other targets that link against turtlelib
# By adding include/ to the include path, this means that files in e.g., include/turtlelib
# can be included with #include"turtlelib/file.hpp"

# enable C++ 17
target_compile_features(turtlelib PUBLIC cxx_std_17) 

# warnings are your friend!
target_compile_options(turtlelib PUBLIC -Wall -Wextra -Wpedantic)

# create the executable target  and link it with the rigid2d library
# It is also possible specify multiple cpp files and they will be linked
# into a single executable (as long as exactly one of these files includes a main() function).
# However, by creating a library (as we are doing here) the library files
# can be compiled once and used
if(NOT CMAKE_CROSSCOMPILING)

# CMake also has the ability to generate doxygen documentation
find_package(Doxygen)
set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md) # Use the readme in your doxygen docs
doxygen_add_docs(doxygen include/ src/ README.md ALL)

# Use the cmake testing functionality. A test is just an executable.
# We use the catch2 unit testing framework
find_package(Catch2 3 REQUIRED)

include(CTest) #//! Fix the can't find Dart issue
add_executable(turtlelib_test tests/tests.cpp tests/circle_tests.cpp)
# add_executable(turtlelib_circle_test tests/circle_tests.cpp)
target_link_libraries(turtlelib_test turtlelib Catch2::Catch2WithMain ${ARMADILLO_LIBRARIES})
# target_link_libraries(turtlelib_circle_test turtlelib Catch2::Catch2WithMain ${ARMADILLO_LIBRARIES})

add_test(NAME Test_of_Turtlelib COMMAND turtlelib_test)


endif()

target_include_directories(turtlelib
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_compile_options(turtlelib PUBLIC -Wall)

install(DIRECTORY include/turtlelib 
        DESTINATION include)

install(TARGETS frame_main turtlelib 
        EXPORT turtlelib-targets)

install(EXPORT turtlelib-targets
        FILE turtlelib-config.cmake
        NAMESPACE turtlelib::
        DESTINATION lib/cmake/${PROJECT_NAME})
