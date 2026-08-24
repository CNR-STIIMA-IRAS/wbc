SET(TARGET_NAME wbc-solvers-eiquadprog)

# pkg_search_module(eiquadprog REQUIRED IMPORTED_TARGET eiquadprog)
find_package(eiquadprog REQUIRED)

set(SOURCES EiquadprogSolver.cpp)
set(HEADERS EiquadprogSolver.hpp)

list(APPEND PKGCONFIG_REQUIRES eiquadprog)
list(APPEND PKGCONFIG_REQUIRES wbc-core)
string (REPLACE ";" " " PKGCONFIG_REQUIRES "${PKGCONFIG_REQUIRES}")

add_library(${TARGET_NAME} SHARED ${SOURCES} ${HEADERS})
message("EIQUADPROG_INCLUDE_DIRS: ${EIQUADPROG_INCLUDE_DIRS}")
target_include_directories(${TARGET_NAME} PUBLIC ${EIQUADPROG_INCLUDE_DIRS})
target_link_libraries(${TARGET_NAME} PUBLIC
                      wbc-core
                      # PkgConfig::eiquadprog)
                      eiquadprog::eiquadprog)

set_target_properties(${TARGET_NAME} PROPERTIES
       VERSION ${PROJECT_VERSION}
       SOVERSION ${API_VERSION})

install(TARGETS ${TARGET_NAME}
        LIBRARY DESTINATION lib)

CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_NAME}.pc.in ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.pc DESTINATION lib/pkgconfig)
INSTALL(FILES ${HEADERS} DESTINATION include/${PROJECT_NAME}/solvers/eiquadprog)

if(WBC_BUILD_TESTING)
    add_subdirectory(test)
endif()
