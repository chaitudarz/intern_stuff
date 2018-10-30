# Important settings:
# CRATE_MODULE:     Name of the module
# CRATE_INSTALL:    Path to the install directory
#
# CRATE_BUILD_BIN:      True for building a binary
# CRATE_BUILD_LIB:      True for building a library
# CRATE_INSTALL_INC:    True for coping the headers from inc/
# CRATE_BUILD_TEST:     True for building a test

# Variables used by crate's CMake:
# CRATE_INCLUDES:   Paths of the include directories
# CRATE_LINK_DIRS:  Paths to the directories to look for libraries

cmake_minimum_required(VERSION 3.2)
set (CMAKE_CXX_STANDARD 14)

# Export all symbols
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

SET(CRATE_INSTALLED_LIBRARIES "")

# This flag adds an empty library for header files, if no lib/rtmaps/bin is created.
SET(CRATE_ADD_EMPTY_INC_LIB ON)

# Default path to catkin python binary scripts
SET(CATKIN_BIN_PY_SCRIPTS_PATH "scripts")

# Project name
project(${CRATE_MODULE} CXX C)

if(${CRATE_BUILD_DOC})
    find_package(Doxygen)
endif(${CRATE_BUILD_DOC})

# Prepare install logs
file(MAKE_DIRECTORY "${CRATE_PACKAGE_ROOT}/.crate/install_logs")
SET(CRATE_RELEASE_INSTALL_LOG "${CRATE_PACKAGE_ROOT}/.crate/install_logs/${CRATE_MODULE}.release")
file(WRITE "${CRATE_RELEASE_INSTALL_LOG}")

# Adjust compiler flags
if(MSVC)
    # Disable optimization for RelWithDebInfo
    SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "/Od /Ob0 /Zi")
    # Show most (useful) warnings, but ignore following warnings:
        # 4100: unreferenced formal parameter
        # 4577: 'noexcept' used with no exception handling mode specified; termination on exception is not guaranteed. Specify /EHsc
        # 4530: C++ exception handler used, but unwind semantics are not enabled. Specify /EHsc
    SET(CMAKE_CXX_FLAGS "/W4 /wd4100 /wd4577 /wd4530")
    # Enable parallel build for Visual Studio
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /MP")
    # Enable C++ Exceptions for Visual Studio
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc")
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /EHsc")
endif(MSVC)

if(CMAKE_COMPILER_IS_GNUCXX)
    # Enable warnings
    SET(CMAKE_CXX_FLAGS "-Wall -Wno-long-long -pedantic" CACHE STRING "compile flags" FORCE)
    # Disable optimization for RelWithDebInfo
    SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-g -DNDEBUG" CACHE STRING "compile flags" FORCE)
endif(CMAKE_COMPILER_IS_GNUCXX)

if( CMAKE_COMPILER_IS_GNUCC )
    # Enable warnings
    SET(CMAKE_C_FLAGS "-Wall -Wno-long-long -pedantic" CACHE STRING "compile flags" FORCE)
    # Disable optimization for RelWithDebInfo
    SET(CMAKE_C_FLAGS_RELWITHDEBINFO "-g -DNDEBUG" CACHE STRING "compile flags" FORCE)
endif( CMAKE_COMPILER_IS_GNUCC )

# Set dependencies
if (NOT CRATE_DEPENDENCIES)
    SET(CRATE_DEPENDENCIES "")
endif(NOT CRATE_DEPENDENCIES)

# Call user specific pre-crate script
if(EXISTS "${CRATE_CURRENT_DIR}/preCrate.cmake")
    message(STATUS "Including preCrate.cmake for ${CRATE_MODULE} ..")
    include("${CRATE_CURRENT_DIR}/preCrate.cmake")
endif(EXISTS "${CRATE_CURRENT_DIR}/preCrate.cmake")


if(${CRATE_USE_QT5})
    list(APPEND CMAKE_PREFIX_PATH "${QT5DIR}\\lib\\cmake")
    set(CMAKE_AUTOMOC ON)
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
    foreach(QT5PART ${QT5_PARTS})
        find_package(${QT5PART} REQUIRED)
        include_directories("${QT5PART}_INCLUDES")
    endforeach(QT5PART)
endif(${CRATE_USE_QT5})


# Find dependencies
foreach(DEP ${CRATE_DEPENDENCIES})
    find_package("${DEP}" REQUIRED)
endforeach(DEP)

# Set include directories
SET(CRATE_INCLUDES ${CRATE_INCLUDES} "${CRATE_INSTALL}/include")
if (${CRATE_BUILD_ROS} OR ${CRATE_BUILD_ROSMSG} OR ${CRATE_LINK_ONLY_ROS})
    # Add install/ros/include only if any ros component is present
    SET(CRATE_INCLUDES ${CRATE_INCLUDES} "${CRATE_INSTALL}/ros/include")
endif (${CRATE_BUILD_ROS} OR ${CRATE_BUILD_ROSMSG} OR ${CRATE_LINK_ONLY_ROS})
if(EXISTS "${CRATE_CURRENT_DIR}/inc")
    SET(CRATE_INCLUDES ${CRATE_INCLUDES} "${CRATE_CURRENT_DIR}/inc")
endif(EXISTS "${CRATE_CURRENT_DIR}/inc")

# Get Sources for binary
if(NOT CRATE_SRCS_BIN AND ${CRATE_BUILD_BIN})
    file(GLOB_RECURSE CRATE_SRCS_BIN
        "${CRATE_SRC_PATH_PREFIX}inc/*.h"
        "${CRATE_SRC_PATH_PREFIX}inc/*.inc"
        "${CRATE_SRC_PATH_PREFIX}src/*.h"
        "${CRATE_SRC_PATH_PREFIX}src/*.inc"
        "${CRATE_SRC_PATH_PREFIX}src/*.cc"
        "${CRATE_SRC_PATH_PREFIX}src/*.cpp"
    )
endif(NOT CRATE_SRCS_BIN AND ${CRATE_BUILD_BIN})

# Get Sources for library
if(NOT CRATE_SRCS_LIB AND ${CRATE_BUILD_LIB})
    file(GLOB_RECURSE CRATE_SRCS_LIB
        "${CRATE_SRC_PATH_PREFIX}inc/*.h"
        "${CRATE_SRC_PATH_PREFIX}inc/*.inc"
        "${CRATE_SRC_PATH_PREFIX}src/*.h"
        "${CRATE_SRC_PATH_PREFIX}src/*.inc"
        "${CRATE_SRC_PATH_PREFIX}src/*.cc"
        "${CRATE_SRC_PATH_PREFIX}src/*.cpp"
    )
endif(NOT CRATE_SRCS_LIB AND ${CRATE_BUILD_LIB})

# Get Sources for includes
if(NOT CRATE_DIR_INC AND ${CRATE_INSTALL_INC})
    file(GLOB_RECURSE CRATE_SRCS_INC
        "${CRATE_SRC_PATH_PREFIX}inc/*.h"
        "${CRATE_SRC_PATH_PREFIX}inc/*.inc"
    )   
    SET(CRATE_DIR_INC "${CRATE_CURRENT_DIR}/inc/")
endif(NOT CRATE_DIR_INC AND ${CRATE_INSTALL_INC})

# Get Sources for test
if(NOT CRATE_SRCS_TEST AND (${CRATE_BUILD_TEST} OR ${CRATE_BUILD_ROSTEST}))
    file(GLOB_RECURSE CRATE_SRCS_TEST
        "${CRATE_SRC_PATH_PREFIX}test/*.h"
        "${CRATE_SRC_PATH_PREFIX}test/*.inc"
        "${CRATE_SRC_PATH_PREFIX}test/*.cc"
        "${CRATE_SRC_PATH_PREFIX}test/*.cpp"
    )
endif(NOT CRATE_SRCS_TEST AND (${CRATE_BUILD_TEST} OR ${CRATE_BUILD_ROSTEST}))

# Get Sources for rtMaps
if(NOT CRATE_SRCS_RTMAPS AND ${CRATE_BUILD_RTMAPS})
    file(GLOB_RECURSE CRATE_SRCS_RTMAPS
        "${CRATE_SRC_PATH_PREFIX}inc/*.h"
        "${CRATE_SRC_PATH_PREFIX}inc/*.inc"
        "${CRATE_SRC_PATH_PREFIX}src/*.h"
        "${CRATE_SRC_PATH_PREFIX}src/*.inc"
        "${CRATE_SRC_PATH_PREFIX}src/*.cc"
        "${CRATE_SRC_PATH_PREFIX}src/*.cpp"
    )
endif(NOT CRATE_SRCS_RTMAPS AND ${CRATE_BUILD_RTMAPS})

# Get Sources for ros
if(NOT CRATE_SRCS_ROS AND ${CRATE_BUILD_ROS})
    file(GLOB_RECURSE CRATE_SRCS_ROS
        "${CRATE_SRC_PATH_PREFIX}inc/*.h"
        "${CRATE_SRC_PATH_PREFIX}inc/*.inc"
        "${CRATE_SRC_PATH_PREFIX}src/*.h"
        "${CRATE_SRC_PATH_PREFIX}src/*.inc"
        "${CRATE_SRC_PATH_PREFIX}src/*.cc"
        "${CRATE_SRC_PATH_PREFIX}src/*.cpp"
    )
endif(NOT CRATE_SRCS_ROS AND ${CRATE_BUILD_ROS})

# Get Sources for mex
if(NOT CRATE_SRCS_MEX AND ${CRATE_BUILD_MEX})

    # message(STATUS "CurDir ${CRATE_CURRENT_DIR} ..")
    file(GLOB_RECURSE CRATE_SRCS_MEX
        "${CRATE_CURRENT_DIR}/*.cpp"
    )

endif(NOT CRATE_SRCS_MEX AND ${CRATE_BUILD_MEX})

# Start Writing Find.. script
file(WRITE "${CRATE_INSTALL}/cmake/Find${CRATE_MODULE}.cmake" "# Autogenerated by CRATE for module ${CRATE_MODULE}.\n\n")
file(WRITE "${CRATE_INSTALL}/cmake/Find${CRATE_MODULE}.cmake.installed" "# Autogenerated by CRATE for module ${CRATE_MODULE}.\n\n")

# Build and install:
include_directories(${CRATE_INCLUDES})

# Set a "fake" target if only headers have to be installed
if(${CRATE_INSTALL_INC})
    if(NOT ${CRATE_BUILD_LIB} AND NOT ${CRATE_BUILD_RTMAPS} AND NOT ${CRATE_BUILD_BIN} AND ${CRATE_ADD_EMPTY_INC_LIB})
        add_library (${CRATE_MODULE} STATIC ${CRATE_SRCS_INC})
        set_target_properties(${CRATE_MODULE} PROPERTIES LINKER_LANGUAGE CXX)
        # Add to dependencies
        SET(CRATE_DEPENDENCIES "${CRATE_DEPENDENCIES};${CRATE_MODULE}")
    endif(NOT ${CRATE_BUILD_LIB} AND NOT ${CRATE_BUILD_RTMAPS} AND NOT ${CRATE_BUILD_BIN} AND ${CRATE_ADD_EMPTY_INC_LIB})
endif(${CRATE_INSTALL_INC})

# Install library
if(${CRATE_BUILD_LIB})
    crate_add_lib(${CRATE_MODULE} "${CRATE_SRCS_LIB}" "${CRATE_INSTALL}/lib" "${CRATE_DEPENDENCIES}")
endif(${CRATE_BUILD_LIB})

# Install binary
if(${CRATE_BUILD_BIN}) 
    crate_add_bin(${CRATE_MODULE} "${CRATE_SRCS_BIN}" "${CRATE_INSTALL}/bin" "${CRATE_DEPENDENCIES}")
endif(${CRATE_BUILD_BIN})

# Install RTMaps
if(${CRATE_BUILD_RTMAPS})
    crate_add_rtmaps("${CRATE_MODULE}" "${CRATE_SRCS_RTMAPS}" "${CRATE_INSTALL}/rtmaps" "${CRATE_DEPENDENCIES}")
endif(${CRATE_BUILD_RTMAPS})

# Install ROSMSG
if(${CRATE_BUILD_ROSMSG})
    crate_add_rosmsg("${CRATE_MODULE}" "${CRATE_INSTALL}/ros" "${CRATE_DEPENDENCIES}")
endif(${CRATE_BUILD_ROSMSG})

# Install ROS
if(${CRATE_BUILD_ROS})
    crate_add_ros("${CRATE_MODULE}_node" "${CRATE_SRCS_ROS}" "${CRATE_INSTALL}/ros" "${CRATE_DEPENDENCIES}")
endif(${CRATE_BUILD_ROS})

# Install MEX
if(${CRATE_BUILD_MEX})
    crate_add_mex(${CRATE_MODULE} "${CRATE_SRCS_MEX}" "${CRATE_INSTALL}/mex" "${CRATE_DEPENDENCIES}")
endif(${CRATE_BUILD_MEX})

# Link only ROS
if(${CRATE_LINK_ONLY_ROS})
    crate_link_ros("${CRATE_MODULE}" "${CRATE_SRCS_ROS}" "${CRATE_INSTALL}/ros" "${CRATE_DEPENDENCIES}")
endif(${CRATE_LINK_ONLY_ROS})

# Install ROSTEST
if(${CRATE_BUILD_ROSTEST})
    crate_add_rostest("${CRATE_MODULE}Test" "${CRATE_SRCS_TEST}" "${CRATE_INSTALL}/ros" "${CRATE_DEPENDENCIES}")
endif(${CRATE_BUILD_ROSTEST})

# Install test
if(${CRATE_BUILD_TEST})
    # require gtest for testing
    find_package("gtest" REQUIRED)
    crate_add_bin("${CRATE_MODULE}Test" "${CRATE_SRCS_TEST}" "${CRATE_INSTALL}/test" "${CRATE_DEPENDENCIES};gtest;gtest_main")
endif(${CRATE_BUILD_TEST})

# Install header
if(${CRATE_INSTALL_INC})
    foreach(inc_file ${CRATE_SRCS_INC})
        if("${CRATE_SRC_PATH_PREFIX}" STREQUAL "../")
            string(REPLACE "${CRATE_CURRENT_DIR}/self/../inc/" "" filename "${inc_file}")
        else("${CRATE_SRC_PATH_PREFIX}" STREQUAL "../")
            string(REPLACE "${CRATE_CURRENT_DIR}/inc/" "" filename "${inc_file}")
        endif("${CRATE_SRC_PATH_PREFIX}" STREQUAL "../")
        #get_filename_component(filename ${inc_file} NAME)
        # install headers in subfolder with module name
        if("${filename}" MATCHES "^${CRATE_MODULE}")
            string(REPLACE "${CRATE_MODULE}/" "" filename "${filename}")
        endif("${filename}" MATCHES "^${CRATE_MODULE}")
        add_custom_command(TARGET "${CRATE_MODULE}" PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different ${inc_file} "${CRATE_INSTALL}/include/${CRATE_MODULE}/${filename}")
        file(APPEND "${CRATE_RELEASE_INSTALL_LOG}" "install/include/${CRATE_MODULE}/${filename}\n")
    endforeach(inc_file)

    #INSTALL( DIRECTORY ${CRATE_DIR_INC} DESTINATION "${CRATE_INSTALL}/include/${CRATE_MODULE}" FILE_PERMISSIONS OWNER_READ GROUP_READ WORLD_READ COMPONENT INC_INSTALL )
endif(${CRATE_INSTALL_INC})


if(${CRATE_BUILD_DOC})
    if(NOT DOXYGEN_FOUND)
        message(WARNING "Doxygen is needed to build the documentation.")
    else(NOT DOXYGEN_FOUND)
        set(DOXY_DOC_DEST_DIR ${CRATE_INSTALL}/doc/${CRATE_MODULE})
        set(DOXY_DOC_INPUT_ROOT_DIRS ${CRATE_CURRENT_DIR})
        set(DOXY_DOC_EXCLUDE_PATTERNS_DIRS ${CMAKE_BINARY_DIR})
        set(DOXY_DOC_COMMON_IMG_PATH ${CRATE_CURRENT_DIR})
        configure_file("${CMAKE_SOURCE_DIR}/.crate/doxyfile.in" "${CMAKE_BINARY_DIR}/doxyfile_${CRATE_MODULE}" @ONLY)
        add_custom_target("${CRATE_MODULE}Doc" COMMAND ${DOXYGEN_EXECUTABLE} "${CMAKE_BINARY_DIR}/doxyfile_${CRATE_MODULE}"
            WORKING_DIRECTORY ${CMAKE_HOME_DIRECTORY}
            COMMENT "Building user's documentation into doxyDoc build dir..."
        )
    endif(NOT DOXYGEN_FOUND)
endif(${CRATE_BUILD_DOC})

if(${CRATE_USE_QT5})
    foreach(QT5PART ${QT5_PARTS})
        find_package(${QT5PART} REQUIRED)
        string(REPLACE "Qt5" "Qt5::" qt5_linker_name "${QT5PART}")
        target_link_libraries(${CRATE_MODULE} "${qt5_linker_name}")
        if(MSVC)
            get_target_property(DEBUG_LOCATION "${qt5_linker_name}" IMPORTED_LOCATION_DEBUG)
            add_custom_command(
                TARGET ${CRATE_MODULE} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy ${DEBUG_LOCATION} "${CRATE_INSTALL}/lib/Debug/${QT5PART}d.dll"
            )        
            get_target_property(RELEASE_LOCATION "${qt5_linker_name}" IMPORTED_LOCATION_RELEASE)
            add_custom_command(
                TARGET ${CRATE_MODULE} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy ${RELEASE_LOCATION} "${CRATE_INSTALL}/lib/${QT5PART}.dll"
            )
        else(MSVC)
            get_target_property(DEBUG_LOCATION "${qt5_linker_name}" IMPORTED_LOCATION_DEBUG)       
            get_target_property(RELEASE_LOCATION "${qt5_linker_name}" IMPORTED_LOCATION_RELEASE)
            add_custom_command(
                TARGET ${CRATE_MODULE} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy ${RELEASE_LOCATION} "${CRATE_INSTALL}/lib/lib${QT5PART}.so"
            )
        endif(MSVC)
    endforeach(QT5PART)
endif(${CRATE_USE_QT5})

# Call user specific post-crate script
if(EXISTS "${CRATE_CURRENT_DIR}/postCrate.cmake")
    message(STATUS "Including postCrate.cmake for ${CRATE_MODULE} ..")
    include("${CRATE_CURRENT_DIR}/postCrate.cmake")
endif(EXISTS "${CRATE_CURRENT_DIR}/postCrate.cmake")
