cmake_minimum_required(VERSION 3.10)

PROJECT(pyoctomap LANGUAGES CXX)

#################
# Find pybind11 #
#################

IF(SKBUILD)
  # Scikit-Build does not add your site-packages to the search path automatically, so we need to add it
  # _or_ the pybind11 specific directory here.
  EXECUTE_PROCESS(
    COMMAND "${PYTHON_EXECUTABLE}" -c "import pybind11; print(pybind11.get_cmake_dir())"
    OUTPUT_VARIABLE _tmp_dir
    OUTPUT_STRIP_TRAILING_WHITESPACE COMMAND_ECHO STDOUT
  )
  LIST(APPEND CMAKE_PREFIX_PATH "${_tmp_dir}")
ENDIF()

FIND_PACKAGE(pybind11 CONFIG REQUIRED)

###########################
# Specify the target name #
###########################

SET(targetname pyoctomap)

################################
# Specify the libraries to use #
################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOctomap.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenGL.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseQGLViewer.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseQt5.cmake)

##########################################
# Specify additional include directories #
##########################################

INCLUDE_DIRECTORIES(${pybind11_INCLUDE_DIRS})

######################
# Specify the target #
######################

PYBIND11_ADD_MODULE(${targetname} SHARED ${targetname}.cpp smg/${targetname}/cpp/${targetname}.pyi)
TARGET_COMPILE_DEFINITIONS(${targetname} PRIVATE VERSION_INFO=${PROJECT_VERSION})

#################################
# Specify the libraries to link #
#################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkOctomap.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkOpenGL.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkQGLViewer.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkQt5.cmake)

#################################
# Specify the things to install #
#################################

INSTALL(TARGETS ${targetname} DESTINATION .)
INSTALL(FILES ${QGLVIEWER_RUNTIMELIBS} DESTINATION .)
