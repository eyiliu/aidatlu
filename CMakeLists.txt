cmake_minimum_required(VERSION 3.22 FATAL_ERROR)
project(aidatlu VERSION 1.0.0 LANGUAGES CXX)


if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/INSTALL CACHE PATH "Default prefix of install directories" FORCE)
endif()
# set(CMAKE_INSTALL_DEFAULT_COMPONENT_NAME devel)

set(CMAKE_CXX_STANDARD 17)

# set(CMAKE_FIND_LIBRARY_SUFFIXES ".a")
# set(CMAKE_LINK_SEARCH_START_STATIC ON)
# set(CMAKE_LINK_SEARCH_END_STATIC ON)

option(BUILD_AIDATLU_STATIC "build aidatlu tool static" OFF)

if(BUILD_AIDATLU_STATIC)
  set(CMAKE_EXE_LINKER_FLAGS    "${CMAKE_EXE_LINKER_FLAGS} -static-libstdc++ -static-libgcc")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -static-libstdc++ -static-libgcc")
endif()
  
set(CMAKE_THREAD_PREFER_PTHREAD ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

if(BUILD_AIDATLU_STATIC)
  set(Boost_USE_STATIC_RUNTIME ON)
  set(Boost_USE_STATIC_LIBS ON)
endif()


cmake_host_system_information(RESULT PLATFORM_ID QUERY DISTRIB_PLATFORM_ID)
if(PLATFORM_ID STREQUAL "platform:el8")
  find_package(Boost 1.66 EXACT REQUIRED COMPONENTS
    system thread filesystem chrono regex program_options unit_test_framework
  )  
elseif(PLATFORM_ID STREQUAL "platform:el9")
  find_package(Boost 1.75 EXACT REQUIRED COMPONENTS
    system thread filesystem chrono regex program_options unit_test_framework
  )
else()
  find_package(Boost REQUIRED COMPONENTS
    system thread filesystem chrono regex program_options unit_test_framework
  )
endif()
 

if(BUILD_AIDATLU_STATIC)
  add_library(cactus_uhal_grammars STATIC IMPORTED)
  set_target_properties(cactus_uhal_grammars PROPERTIES
    IMPORTED_LOCATION  /opt/cactus/lib/libcactus_uhal_grammars.a
    INTERFACE_INCLUDE_DIRECTORIES /opt/cactus/include
    )
  add_library(cactus_uhal_log STATIC IMPORTED)
  set_target_properties(cactus_uhal_log PROPERTIES
    IMPORTED_LOCATION  /opt/cactus/lib/libcactus_uhal_log.a
    INTERFACE_INCLUDE_DIRECTORIES /opt/cactus/include
    )
  add_library(cactus_uhal_tests STATIC IMPORTED)
  set_target_properties(cactus_uhal_tests PROPERTIES
    IMPORTED_LOCATION  /opt/cactus/lib/libcactus_uhal_tests.a
    INTERFACE_INCLUDE_DIRECTORIES /opt/cactus/include
    )
  add_library(cactus_uhal_uhal STATIC IMPORTED)
  set_target_properties(cactus_uhal_uhal PROPERTIES
    IMPORTED_LOCATION  /opt/cactus/lib/libcactus_uhal_uhal.a
    INTERFACE_INCLUDE_DIRECTORIES /opt/cactus/include
    )
else()
  add_library(cactus_uhal_grammars SHARED IMPORTED)
  set_target_properties(cactus_uhal_grammars PROPERTIES
    IMPORTED_LOCATION  /opt/cactus/lib/libcactus_uhal_grammars.so
    INTERFACE_INCLUDE_DIRECTORIES /opt/cactus/include
    )
  add_library(cactus_uhal_log SHARED IMPORTED)
  set_target_properties(cactus_uhal_log PROPERTIES
    IMPORTED_LOCATION  /opt/cactus/lib/libcactus_uhal_log.so
    INTERFACE_INCLUDE_DIRECTORIES /opt/cactus/include
    )
  add_library(cactus_uhal_tests SHARED IMPORTED)
  set_target_properties(cactus_uhal_tests PROPERTIES
    IMPORTED_LOCATION  /opt/cactus/lib/libcactus_uhal_tests.so
    INTERFACE_INCLUDE_DIRECTORIES /opt/cactus/include
    )
  add_library(cactus_uhal_uhal SHARED IMPORTED)
  set_target_properties(cactus_uhal_uhal PROPERTIES
    IMPORTED_LOCATION  /opt/cactus/lib/libcactus_uhal_uhal.so
    INTERFACE_INCLUDE_DIRECTORIES /opt/cactus/include
    )
endif()

add_library(cactus INTERFACE)
target_link_libraries(cactus INTERFACE
  cactus_uhal_uhal cactus_uhal_grammars cactus_uhal_tests cactus_uhal_log
  Boost::chrono Boost::filesystem Boost::program_options
  Boost::unit_test_framework Boost::regex Boost::system Boost::thread
  Threads::Threads rt)
target_include_directories(cactus INTERFACE /opt/cactus/include)

set(AIDA_TLU_REG_XML ${CMAKE_SOURCE_DIR}/doc/aida_tlu_address-fw_version_14.xml)
set(AIDA_TLU_REG_HH  ${CMAKE_BINARY_DIR}/aida_tlu_address-fw_version_14.hh)
if( (NOT EXISTS ${AIDA_TLU_REG_HH}) OR (${AIDA_TLU_REG_XML} IS_NEWER_THAN ${AIDA_TLU_REG_HH}) )
  set(AIDA_TLU_REG_HH_DELIM "~~~cpp~raw~~~")
  file(READ ${AIDA_TLU_REG_XML} AIDA_TLU_REG_XML_CONTENT)
  set(AIDA_TLU_REG_XML_CONTENT "R\"${AIDA_TLU_REG_HH_DELIM}(${AIDA_TLU_REG_XML_CONTENT})${AIDA_TLU_REG_HH_DELIM}\"")
  file(WRITE ${AIDA_TLU_REG_HH} "${AIDA_TLU_REG_XML_CONTENT}")
endif()

set(AIDA_TLU_CLK_TXT ${CMAKE_SOURCE_DIR}/doc/aida_tlu_clk_config.txt)
set(AIDA_TLU_CLK_HH  ${CMAKE_BINARY_DIR}/aida_tlu_clk_config.hh)
if( (NOT EXISTS ${AIDA_TLU_CLK_HH}) OR (${AIDA_TLU_CLK_TXT} IS_NEWER_THAN ${AIDA_TLU_CLK_HH}) )
  set(AIDA_TLU_CLK_HH_DELIM "~~~cpp~raw~~~")
  file(READ ${AIDA_TLU_CLK_TXT} AIDA_TLU_CLK_TXT_CONTENT)
  set(AIDA_TLU_CLK_TXT_CONTENT "R\"${AIDA_TLU_CLK_HH_DELIM}(${AIDA_TLU_CLK_TXT_CONTENT})${AIDA_TLU_CLK_HH_DELIM}\"")
  file(WRITE ${AIDA_TLU_CLK_HH} "${AIDA_TLU_CLK_TXT_CONTENT}")
endif()


aux_source_directory(src LIB_SRC)
add_library(aidatlu STATIC ${LIB_SRC})
target_link_libraries(aidatlu PRIVATE cactus)
target_include_directories(aidatlu
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}>
  )

set(LIB_PUBLIC_HEADERS include/AidaTluController.hh)
set_target_properties(aidatlu PROPERTIES PUBLIC_HEADER "${LIB_PUBLIC_HEADERS}")

#target_link_libraries(aidatlu PRIVATE -Wl,--whole-archive ${CACTUS_LIBRARIES} -Wl,--no-whole-archive)

if(NOT BUILD_AIDATLU_STATIC)
  set_target_properties(aidatlu PROPERTIES INSTALL_RPATH /opt/cactus/lib)
endif()
  
add_subdirectory(exe)

# install-tree

set(CONF_INCLUDE_DIRS "\${CMAKE_CURRENT_LIST_DIR}/../include")
configure_file(cmake/aidatluConfig.cmake.in         ${PROJECT_BINARY_DIR}/cmake_install/aidatluConfig.cmake @ONLY)
configure_file(cmake/aidatluConfigVersion.cmake.in  ${PROJECT_BINARY_DIR}/cmake_install/aidatluConfigVersion.cmake @ONLY)
install(FILES
  ${PROJECT_BINARY_DIR}/cmake_install/aidatluConfig.cmake
  ${PROJECT_BINARY_DIR}/cmake_install/aidatluConfigVersion.cmake
  DESTINATION cmake COMPONENT devel)

install(TARGETS aidatlu cactus
  EXPORT ${PROJECT_NAME}Targets
  RUNTIME       DESTINATION bin      COMPONENT runtime
  LIBRARY       DESTINATION lib      COMPONENT runtime
  ARCHIVE       DESTINATION lib      COMPONENT devel
  PUBLIC_HEADER DESTINATION include  COMPONENT devel
  RESOURCE      DESTINATION resource COMPONENT runtime
  )

install(EXPORT ${PROJECT_NAME}Targets
  DESTINATION cmake
  )

install(CODE "MESSAGE(\"project is installed in ${CMAKE_INSTALL_PREFIX} .\")")

# build-tree
set(CONF_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include ${PROJECT_BINARY_DIR})
configure_file(cmake/aidatluConfig.cmake.in          ${PROJECT_BINARY_DIR}/cmake/aidatluConfig.cmake @ONLY)
configure_file(cmake/aidatluConfigVersion.cmake.in   ${PROJECT_BINARY_DIR}/cmake/aidatluConfigVersion.cmake @ONLY)

export(TARGETS aidatlu cactus FILE ${PROJECT_BINARY_DIR}/cmake/${PROJECT_NAME}Targets.cmake)
export(PACKAGE ${PROJECT_NAME})
