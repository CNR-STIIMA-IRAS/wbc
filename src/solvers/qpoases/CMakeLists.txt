SET(TARGET_NAME wbc-solvers-qpoases)

# qpOASES is provided as a CMake package in the ROS workspace.

set(SOURCES QPOasesSolver.cpp)
set(HEADERS QPOasesSolver.hpp)

list(APPEND PKGCONFIG_REQUIRES wbc-core)
string (REPLACE ";" " " PKGCONFIG_REQUIRES "${PKGCONFIG_REQUIRES}")
set(PKGCONFIG_CFLAGS "-I${qpOASES_INCLUDE_DIR}")
set(PKGCONFIG_LIBS "${qpOASES_LIBRARY}")

add_library(${TARGET_NAME} SHARED ${SOURCES} ${HEADERS})
target_include_directories(${TARGET_NAME} PUBLIC ${qpOASES_INCLUDE_DIR})
target_link_libraries(${TARGET_NAME} PUBLIC
                      wbc-core
                      ${qpOASES_LIBRARY})

set_target_properties(${TARGET_NAME} PROPERTIES
       VERSION ${PROJECT_VERSION}
       SOVERSION ${API_VERSION})

install(TARGETS ${TARGET_NAME}
        LIBRARY DESTINATION lib)

CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_NAME}.pc.in ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.pc DESTINATION lib/pkgconfig)
INSTALL(FILES ${HEADERS} DESTINATION include/${PROJECT_NAME}/solvers/qpoases)

if(WBC_BUILD_TESTING)
    add_subdirectory(test)
endif()
