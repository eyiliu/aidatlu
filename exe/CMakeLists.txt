add_executable(aidatluTool aidatluTool.cxx)

if(BUILD_AIDATLU_STATIC)
  target_link_libraries(aidatluTool PRIVATE -static aidatlu)
else()
  target_link_libraries(aidatluTool PRIVATE aidatlu)
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
