add_executable(aidatluTool aidatluTool.cxx)
target_link_libraries(aidatluTool PRIVATE aidatlu)
if(NOT BUILD_AIDATLU_STATIC)
  set_target_properties(aidatluTool PROPERTIES INSTALL_RPATH /opt/cactus/lib)
  set_target_properties(aidatluTool PROPERTIES BUILD_RPATH /opt/cactus/lib)
endif()

install(TARGETS aidatluTool
  EXPORT ${PROJECT_NAME}Targets
  RUNTIME       DESTINATION bin      COMPONENT runtime
  LIBRARY       DESTINATION lib      COMPONENT runtime
  ARCHIVE       DESTINATION lib      COMPONENT devel
  PUBLIC_HEADER DESTINATION include  COMPONENT devel
  RESOURCE      DESTINATION resource COMPONENT runtime
  )
