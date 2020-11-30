#################
# LinkQt5.cmake #
#################

TARGET_LINK_LIBRARIES(${targetname} ${QT_LIBRARIES})

IF(MSVC_IDE)
  SET(Qt5_BIN_DIR "${Qt5_DIR}/../../../bin")
  SET(FILES_TO_COPY "Qt5Core.dll;Qt5Cored.dll;Qt5Gui.dll;Qt5Guid.dll;Qt5OpenGL.dll;Qt5OpenGLd.dll;Qt5Widgets.dll;Qt5Widgetsd.dll;Qt5Xml.dll;Qt5Xmld.dll")
  FOREACH(FILE_TO_COPY IN LISTS FILES_TO_COPY)
    ADD_CUSTOM_COMMAND(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${Qt5_BIN_DIR}/${FILE_TO_COPY}" "$<TARGET_FILE_DIR:${targetname}>")
  ENDFOREACH()
ENDIF()
