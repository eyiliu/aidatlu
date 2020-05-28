
# add_executable(mytlu tlu_test_main.cxx)
# target_link_libraries(mytlu PRIVATE aidatlu)

add_executable(mytlu_ctrl AidaTluController.cxx)
target_link_libraries(mytlu_ctrl PRIVATE aidatlu)
if(NOT BUILD_AIDATLU_STATIC)
  set_target_properties(mytlu_ctrl PROPERTIES INSTALL_RPATH /opt/cactus/lib)
endif()

install(TARGETS mytlu_ctrl
  EXPORT ${PROJECT_NAME}Targets
  RUNTIME       DESTINATION bin      COMPONENT runtime
  LIBRARY       DESTINATION lib      COMPONENT runtime
  ARCHIVE       DESTINATION lib      COMPONENT devel
  PUBLIC_HEADER DESTINATION include  COMPONENT devel
  RESOURCE      DESTINATION resource COMPONENT runtime
  )
