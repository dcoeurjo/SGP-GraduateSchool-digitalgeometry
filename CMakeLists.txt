project(SGP-GraduateSchool-dgtal)
CMAKE_MINIMUM_REQUIRED(VERSION 3.11)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(deps)

##DGtal Specific settings
include("${PROJECT_BINARY_DIR}/deps/DGtal/DGtalConfig.cmake")
include_directories("${DGTAL_INCLUDE_DIRS}")

set(SRCs
   simpleTest
   dt)

foreach(src ${SRCs})
  add_executable(${src} ${src}.cpp)
  target_link_libraries(${src} polyscope DGtal ${DGTAL_LIBRARIES})
endforeach()


