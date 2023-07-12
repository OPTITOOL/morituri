
cmake_minimum_required(VERSION 3.10)

project(comm2osm)

# cmake custom module path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# set c++ standard
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ADD_DEFINITIONS(-DBOOST_LOG_DYN_LINK)
message(STATUS "Build ${CMAKE_BUILD_TYPE}")

# compiler flags
IF(CMAKE_BUILD_TYPE MATCHES Debug)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic -g  -gdwarf-2  -fno-omit-frame-pointer")
ELSEIF(CMAKE_BUILD_TYPE MATCHES Release)
  add_definitions(-DNDEBUG)

  # LTO produces slow binary (maybe libgeos)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -Wextra -pedantic")
ENDIF()

set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_STATIC_RUNTIME ON)

find_package(Boost REQUIRED COMPONENTS system filesystem locale log program_options)
find_package(GDAL REQUIRED)
find_package(geos REQUIRED)
find_path(LIBOSMIUM_INCLUDE_DIRS "osmium/area/assembler.hpp")
find_package(shp CONFIG REQUIRED)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# find source and header filesfile
set(MAIN_CPP_FILES ${PROJECT_SOURCE_DIR}/comm2osm.cpp)
file(GLOB_RECURSE CPP_FILES ${PROJECT_SOURCE_DIR}/plugins/*.cpp)
file(GLOB_RECURSE INCLUDE_FILES ${PROJECT_SOURCE_DIR}/*.h)

include_directories(${GDAL_INCLUDE_DIRS} ${GEOS_INCLUDE_DIR} ${OSMIUM_INCLUDE_DIRS} ${SHP_INCLUDE_DIR})

add_executable(${PROJECT_NAME} ${MAIN_CPP_FILES} ${CPP_FILES})

target_link_libraries(${PROJECT_NAME} PRIVATE Boost::boost Boost::log Boost::locale Boost::program_options
  PRIVATE GEOS::geos
  PRIVATE ${LIBOSMIUM_INCLUDE_DIRS}
  PRIVATE GDAL::GDAL
  PRIVATE shp
  PRIVATE expat bz2)