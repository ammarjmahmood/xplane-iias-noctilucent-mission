cmake_minimum_required(VERSION 3.16)
project(NoctilucentMission VERSION 2.0 LANGUAGES CXX)

set(XPLANE_SDK_ROOT "${CMAKE_SOURCE_DIR}/third_party/sdk/SDK" CACHE PATH "X-Plane SDK root")
if(NOT EXISTS "${XPLANE_SDK_ROOT}/CHeaders/XPLM/XPLMPlugin.h")
  message(FATAL_ERROR "Download X-Plane SDK 4.3.0 into third_party/sdk/SDK (see README.md)")
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Simulation core (no X-Plane dependency) shared by the plugin and tests.
add_library(mission_core STATIC src/flight.cpp src/autopilot.cpp)
target_compile_features(mission_core PUBLIC cxx_std_17)
target_include_directories(mission_core PUBLIC src)
set_target_properties(mission_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
if(NOT MSVC)
  target_compile_options(mission_core PRIVATE -Wall -Wextra)
endif()

add_library(noctilucent MODULE src/plugin.cpp src/avionics.cpp)
target_include_directories(noctilucent PRIVATE "${XPLANE_SDK_ROOT}/CHeaders/XPLM")
target_compile_definitions(noctilucent PRIVATE XPLM200=1 XPLM210=1 XPLM300=1 XPLM301=1 XPLM400=1 XPLM410=1)
if(WIN32)
  # Windows: win.xpl, linked against the SDK's XPLM_64.lib and OpenGL.
  target_compile_definitions(noctilucent PRIVATE IBM=1 _USE_MATH_DEFINES NOMINMAX)
  target_link_libraries(noctilucent PRIVATE mission_core "${XPLANE_SDK_ROOT}/Libraries/Win/XPLM_64.lib" opengl32)
  set(XPL_NAME "win")
  if(MSVC)
    set_property(TARGET noctilucent mission_core PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded")
  else()
    target_link_options(noctilucent PRIVATE -static)
  endif()
elseif(APPLE)
  target_compile_definitions(noctilucent PRIVATE APL=1)
  target_link_libraries(noctilucent PRIVATE mission_core "-framework OpenGL"
    "-F${XPLANE_SDK_ROOT}/Libraries/Mac" "-framework XPLM")
  set(XPL_NAME "mac")
else()
  # Linux: XPLM symbols are resolved by the simulator when it loads the plugin.
  target_compile_definitions(noctilucent PRIVATE LIN=1)
  target_link_libraries(noctilucent PRIVATE mission_core GL)
  target_compile_options(noctilucent PRIVATE -Wall -Wextra -fvisibility=hidden)
  set(XPL_NAME "lin")
endif()
set_target_properties(noctilucent PROPERTIES PREFIX "" OUTPUT_NAME "${XPL_NAME}" SUFFIX ".xpl"
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/NoctilucentClouds/64"
  LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/NoctilucentClouds/64"
  RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/NoctilucentClouds/64")

add_custom_target(package ALL
  COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_SOURCE_DIR}/assets" "${CMAKE_BINARY_DIR}/NoctilucentClouds/assets"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_SOURCE_DIR}/README.md" "${CMAKE_BINARY_DIR}/NoctilucentClouds/README.md"
  DEPENDS noctilucent
  COMMENT "Assembling X-Plane plugin folder")

include(CTest)
if(BUILD_TESTING)
  add_executable(mission_sim tests/mission_sim.cpp)
  target_link_libraries(mission_sim PRIVATE mission_core)
  target_compile_definitions(mission_sim PRIVATE _USE_MATH_DEFINES)
  add_test(NAME full_mission_autoflight COMMAND mission_sim)
endif()
