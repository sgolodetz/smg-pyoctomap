################
# UseQt5.cmake #
################

FIND_PACKAGE(Qt5 COMPONENTS Core Gui OpenGL Widgets Xml HINTS "$ENV{SMGLIB_Qt5_DIR}" REQUIRED)

SET(QT_LIBRARIES Qt5::Core Qt5::Gui Qt5::OpenGL Qt5::Widgets Qt5::Xml)

INCLUDE_DIRECTORIES(
"${Qt5Core_INCLUDE_DIRS}"
"${Qt5Gui_INCLUDE_DIRS}"
"${Qt5OpenGL_INCLUDE_DIRS}"
"${Qt5Widgets_INCLUDE_DIRS}"
"${Qt5Xml_INCLUDE_DIRS}"
)
