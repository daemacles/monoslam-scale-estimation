cmake_minimum_required (VERSION 2.6)
set (PROJECT_NAME MyProject)
project (${PROJECT_NAME})

find_package (Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR} /usr/include/suitesparse)
find_package (cpp-matplotlib REQUIRED)

if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE Release)
endif (NOT CMAKE_BUILD_TYPE)

## Set optional features.  This will show up as a preprocessor variable
option (USE_GLIBCXX_DEBUG 
  "Use the _GLIBCXX_DEBUG macro for nicer debugging and error checking of the c++ standard library.  Warning, it breaks ABI compatibility so don't pass container instantiations between translation units that don't have the same debug mode."
  OFF)
## USE_MY_LIBRARY in source.
#option (USE_MY_LIBRARY
#        "Use the provided library" ON)

SET (CMAKE_C_COMPILER             "/usr/bin/clang-3.5")
SET (CMAKE_CXX_COMPILER             "/usr/bin/clang++-3.5")

## Support for Clang's CompilationDatabase system
set (CMAKE_EXPORT_COMPILE_COMMANDS 1)

## Compile and create a library.  STATIC is default unless BUILD_SHARED_LIBS
## is on.
#add_library (LibraryName src/sources.cc)

#if (USE_MY_LIBRARY)
  ## Search for include files here as well
  #include_directories ("{PROJECT_SOURCE_DIR}/some_sub_path")

  ## Run Cmake also in this dir
  #add_subdirectory (some_sub_path)

  #set (EXTRA_LIBS ${EXTRA_LIBS} LibraryName)
#endif (USE_MY_LIBRARY)

string(TOUPPER ${CMAKE_BUILD_TYPE} DEBUG_MODE)
if (DEBUG_MODE MATCHES "DEBUG")
  set (PCH_FLAGS -O0)
  set (DEBUG_LIB "_d")
elseif (DEBUG_MODE MATCHES "RELEASE")
  set (PCH_FLAGS -O3)
  set (DEBUG_LIB "")
elseif (DEBUG_MODE MATCHES "RELWITHDEBINFO")
  set (PCH_FLAGS -O2 -g)
  set (DEBUG_LIB "_d")
elseif (DEBUG_MODE MATCHES "MINSIZEREL")
  set (PCH_FLAGS -Os)
  set (DEBUG_LIB "")
endif (DEBUG_MODE MATCHES "DEBUG")

set (LIBS 
  ${cpp-matplotlib_LIBS} 
  g2o_core${DEBUG_LIB}
  g2o_solver_pcg${DEBUG_LIB}
  g2o_solver_cholmod${DEBUG_LIB}
  g2o_types_slam2d${DEBUG_LIB}
  g2o_stuff${DEBUG_LIB}
  cholmod
  )

function(add_pch base)
  add_custom_command (OUTPUT ${base}.hpp.pch
    COMMAND ${CMAKE_CXX_COMPILER} 
    ARGS -x c++-header -std=c++11 -I${EIGEN3_INCLUDE_DIR} ${PCH_FLAGS}
    -I/usr/include/suitesparse
    ${CMAKE_HOME_DIRECTORY}/src/${base}.hpp 
    -o ${base}.hpp.pch 
    DEPENDS ${CMAKE_HOME_DIRECTORY}/src/${base}.hpp
    VERBATIM
    )
  set_source_files_properties (src/${base}.cc
    PROPERTIES 
    COMPILE_FLAGS "-include ${base}.hpp"
    OBJECT_DEPENDS ${base}.hpp.pch
    )
endfunction()

add_pch (gps.hpp)

add_executable (gps
  src/gps.cc
  src/template_instances.cc
  ) 
target_link_libraries (gps ${LIBS})

# Compile flags
SET (CMAKE_C_FLAGS                "-Wall -std=c11 -Wextra -Werror")
SET (CMAKE_C_FLAGS_DEBUG          "${CMAKE_CFLAGS} -g")
SET (CMAKE_C_FLAGS_MINSIZEREL     "${CMAKE_CFLAGS} -Os -DNDEBUG")
SET (CMAKE_C_FLAGS_RELEASE        "${CMAKE_CFLAGS} -O3 -DNDEBUG")
SET (CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_CFLAGS} -O2 -g")

SET (CMAKE_CXX_FLAGS                "-Wall -std=c++11 -Wextra -Werror")
SET (CMAKE_CXX_FLAGS_DEBUG          "${CMAKE_CXXFLAGS} -g")
SET (CMAKE_CXX_FLAGS_MINSIZEREL     "${CMAKE_CXXFLAGS} -Os -DNDEBUG")
SET (CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXXFLAGS} -O2 -g")
SET (CMAKE_CXX_FLAGS_RELEASE        "${CMAKE_CXXFLAGS} -O3 -DNDEBUG")

if (USE_GLIBCXX_DEBUG)
  # Prefer using G++ for debug mode because the GLIBCXX_DEBUG flag for pretty
  # printing collections in GDB doesn't seem to work with clang.
  string(TOUPPER ${CMAKE_BUILD_TYPE} DEBUG_MODE)
  if (DEBUG_MODE MATCHES "DEBUG")
    set (CMAKE_CXX_COMPILER "g++")
    set (CMAKE_CXX_FLAGS_DEBUG          "${CMAKE_CXXFLAGS} -g -D_GLIBCXX_DEBUG")
  elseif (DEBUG_MODE MATCHES "RELWITHDEBINFO")
    set (CMAKE_CXX_COMPILER "g++")
    set (CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXXFLAGS} -O2 -g -D_GLIBCXX_DEBUG")
  endif (DEBUG_MODE MATCHES "DEBUG")
endif (USE_GLIBCXX_DEBUG)


