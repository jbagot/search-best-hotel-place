cmake_minimum_required(VERSION 3.5)
# Project name
project(repo_cpp)

#########################################################
# FIND GLUT
#########################################################
find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
add_definitions(${GLUT_DEFINITIONS})
if(NOT GLUT_FOUND)
    message(ERROR " GLUT not found!")
endif(NOT GLUT_FOUND)
#########################################################
# FIND OPENGL
#########################################################
find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})
if(NOT OPENGL_FOUND)
    message(ERROR " OPENGL not found!")
endif(NOT OPENGL_FOUND)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -std=c++11 -lGL -lglut -lGLU -lGLEW")

#set(SOURCE_FILES src/dijkstra.cpp src/main.cpp)

include_directories(include)

#set(JSON_HEADERS include/json/json.h include/json/json-forwards.h)
set(SOURCE_FILES src/main.cpp include/pugiXML/pugixml.hpp include/pugiXML/pugiconfig.hpp include/pugiXML/pugixml.cpp
        src/Graph.cpp src/Graph.h src/Node.cpp src/Node.h src/Point.cpp src/Point.h src/Edge.cpp src/Edge.h include/Framework.h include/Framework.cpp)
#set(SOURCE_FILES src/main.cpp include/json/json.h include/json/json-forwards.h include/jsoncpp.cpp src/Graph.cpp src/Graph.h src/Node.cpp src/Node.h)

#add_library(json include/jsoncpp.cpp JSON_HEADERS)
add_executable(repo_cpp ${SOURCE_FILES})


target_link_libraries(repo_cpp ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})