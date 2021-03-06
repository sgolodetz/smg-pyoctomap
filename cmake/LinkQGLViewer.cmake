#######################
# LinkQGLViewer.cmake #
#######################

TARGET_LINK_LIBRARIES(${targetname} PRIVATE ${QGLVIEWER_LIBRARY})

IF(WIN32)
  SET(QGLVIEWER_RUNTIMELIBS "${QGLVIEWER_DIR}/QGLViewer2.dll" "${QGLVIEWER_DIR}/QGLViewerd2.dll")
ENDIF()

FOREACH(RUNTIMELIB ${QGLVIEWER_RUNTIMELIBS})
  ADD_CUSTOM_COMMAND(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different ${RUNTIMELIB} "$<TARGET_FILE_DIR:${targetname}>")
ENDFOREACH()
