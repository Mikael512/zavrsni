cmake_minimum_required(VERSION 3.5.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_srvs
  tesseract_monitoring
  tesseract_rosutils
)
find_package(tesseract_environment REQUIRED)
find_package(tesseract_command_language REQUIRED)
find_package(tesseract_motion_planners REQUIRED)
find_package(tesseract_process_managers REQUIRED)
find_package(tesseract_collision REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_support REQUIRED)
find_package(trajopt_sqp REQUIRED)
find_package(trajopt_ifopt REQUIRED)
find_package(
  PCL REQUIRED
  COMPONENTS core
             features
             filters
             io
             segmentation
             surface)

if(NOT TARGET console_bridge::console_bridge)
  add_library(console_bridge::console_bridge INTERFACE IMPORTED)
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                                                  ${console_bridge_INCLUDE_DIRS})
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_LINK_LIBRARIES ${console_bridge_LIBRARIES})
else()
  get_target_property(CHECK_INCLUDE_DIRECTORIES console_bridge::console_bridge INTERFACE_INCLUDE_DIRECTORIES)
  if(NOT ${CHECK_INCLUDE_DIRECTORIES})
    set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                                                    ${console_bridge_INCLUDE_DIRS})
  endif()
endif()


catkin_package()
tesseract_variables()

macro(add_example test_name test_file)
  add_executable(${test_name} ${test_file})
  target_link_libraries(${test_name}
    zavrsni
    tesseract::tesseract_environment
    ${catkin_LIBRARIES})
  target_compile_options(${test_name} PRIVATE ${TESSERACT_COMPILE_OPTIONS})
  target_clang_tidy(${test_name} ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_cxx_version(${test_name} PUBLIC VERSION ${TESSERACT_CXX_VERSION})
  target_include_directories(${test_name} PRIVATE
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:include>")
  target_include_directories(${test_name} SYSTEM PRIVATE
      ${catkin_INCLUDE_DIRS})
endmacro()

add_example(${PROJECT_NAME}_program1_node src/program1_node.cpp) #dodao

add_library(
  ${PROJECT_NAME}
  src/program1.cpp) #dodao
target_link_libraries(
  ${PROJECT_NAME}
  tesseract::tesseract_environment
  tesseract::tesseract_process_managers
  console_bridge::console_bridge
  ${PCL_LIBRARIES})

target_compile_options(${PROJECT_NAME} PUBLIC ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME} ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION ${TESSERACT_CXX_VERSION})
foreach(DEF ${PCL_DEFINITIONS})
  string(STRIP ${DEF} DEF)
  if(NOT
     "${DEF}"
     STREQUAL
     "")
    string(
      SUBSTRING "${DEF}"
                0
                2
                DEF_PREFIX)
    if("${DEF_PREFIX}" STREQUAL "-m")
      string(
        REPLACE " "
                ";"
                DEF
                ${DEF})
      foreach(OPTION_DEF ${DEF})
        target_compile_options(${PROJECT_NAME} PUBLIC ${OPTION_DEF})
      endforeach()
    else()
      target_compile_definitions(${PROJECT_NAME} PUBLIC ${DEF})
    endif()
  endif()
endforeach()
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})

macro(add_example example_name example_file)
  add_executable(${example_name} ${example_file})
  target_link_libraries(${example_name} ${PROJECT_NAME} tesseract::tesseract_support)
  target_compile_options(${example_name} PRIVATE ${TESSERACT_COMPILE_OPTIONS})
  target_clang_tidy(${example_name} ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_cxx_version(${example_name} PUBLIC VERSION ${TESSERACT_CXX_VERSION})
  target_include_directories(${example_name} PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                     "$<INSTALL_INTERFACE:include>")
endmacro()



#add_example(${PROJECT_NAME}_program1 src/program1_node_extra.cpp) # dodao



# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

configure_package(
  NAMESPACE tesseract
  TARGETS ${PROJECT_NAME}
          #${PROJECT_NAME}_program1 # dodao
          DEPENDENCIES
          tesseract_environment
          tesseract_command_language
          tesseract_motion_planners
          tesseract_process_managers
          tesseract_collision
          tesseract_common
          tesseract_support
          trajopt_sqp
          trajopt_ifopt
          "PCL REQUIRED COMPONENTS core features filters io segmentation surface")
