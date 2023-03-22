set(${PY_NAME}_SOURCES
    main.cpp
    robot_designer.cpp
    ocp.cpp
    mpc.cpp)

add_library(${PY_NAME}_pywrap SHARED ${${PY_NAME}_SOURCES})
target_link_libraries(${PY_NAME}_pywrap PUBLIC ${PROJECT_NAME} eigenpy::eigenpy)
target_include_directories(${PY_NAME}_pywrap SYSTEM
                           PRIVATE ${PYTHON_INCLUDE_DIR})
if (NOT APPLE)
  set_target_properties(
    ${PY_NAME}_pywrap
    PROPERTIES INSTALL_RPATH "\$ORIGIN/../../.."
               LIBRARY_OUTPUT_DIRECTORY ${PY_NAME}
               PREFIX ""
               SUFFIX ${PYTHON_EXT_SUFFIX})
else()
  set_target_properties(
    ${PY_NAME}_pywrap
    PROPERTIES INSTALL_RPATH "${CMAKE_INSTALL_RPATH}"
               LIBRARY_OUTPUT_DIRECTORY ${PY_NAME}
               PREFIX ""
               SUFFIX ${PYTHON_EXT_SUFFIX})
endif()
target_link_boost_python(${PY_NAME}_pywrap PRIVATE)
install(TARGETS ${PY_NAME}_pywrap DESTINATION ${${PY_NAME}_INSTALL_DIR})

set(${PY_NAME}_PYTHON
    __init__.py
)

foreach(python ${${PY_NAME}_PYTHON})
  python_install_on_site(${PY_NAME} ${python})
endforeach()
