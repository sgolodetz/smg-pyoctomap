######################
# LinkQGLViewer.cmake #
######################

TARGET_LINK_LIBRARIES(${targetname} ${QGLVIEWER_LIBRARY})

IF(MSVC_IDE)
  ADD_CUSTOM_COMMAND(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${QGLVIEWER_DIR}/QGLViewer2.dll" "$<TARGET_FILE_DIR:${targetname}>")
  ADD_CUSTOM_COMMAND(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${QGLVIEWER_DIR}/QGLViewerd2.dll" "$<TARGET_FILE_DIR:${targetname}>")
ENDIF()
