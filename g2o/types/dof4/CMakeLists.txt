add_library(types_dof4 ${G2O_LIB_TYPE}
  g2o_types_dof4_api.h
  types_dof4.cpp types_dof4.h
  DOF4.h
  EdgeDOF4.cpp EdgeDOF4.h
  VertexDOF4.cpp VertexDOF4.h
)

set_target_properties(types_dof4 PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types_dof4)
if (APPLE)
  set_target_properties(types_dof4 PROPERTIES INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
endif()
target_link_libraries(types_dof4 core)
if(G2O_HAVE_OPENGL)
  target_link_libraries(types_dof4 opengl_helper ${OPENGL_gl_LIBRARY} )
endif()

install(TARGETS types_dof4
  EXPORT ${G2O_TARGETS_EXPORT_NAME}
  RUNTIME DESTINATION ${RUNTIME_DESTINATION}
  LIBRARY DESTINATION ${LIBRARY_DESTINATION}
  ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
  INCLUDES DESTINATION ${INCLUDES_DESTINATION}
)

file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
install(FILES ${headers} DESTINATION ${INCLUDES_INSTALL_DIR}/types/dof4)
