
# add_executable(mytlu tlu_test_main.cxx)
# target_link_libraries(mytlu PRIVATE aidatlu)

add_executable(aidatluTool AidaTluController.cxx)
target_link_libraries(aidatluTool PRIVATE aidatlu)

install(TARGETS aidatluTool
  EXPORT ${PROJECT_NAME}Targets
  RUNTIME       DESTINATION bin      COMPONENT runtime
  LIBRARY       DESTINATION lib      COMPONENT runtime
  ARCHIVE       DESTINATION lib      COMPONENT devel
  PUBLIC_HEADER DESTINATION include  COMPONENT devel
  RESOURCE      DESTINATION resource COMPONENT runtime
  )
